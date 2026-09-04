#!/usr/bin/env python3
"""行为树 Web 实时看板：树状图可视化 + 手动反馈注入（实机无反馈时保树运行）。

数据源：run_real_bt.py（实机）/ bt_mock_world.py（联调模拟）发布的
BtStateArray。本进程订阅 bt_state 缓存最新快照，并用标准库
ThreadingHTTPServer 提供页面与 JSON（无 rosbridge / 无外部 CDN，离线可用）。

注入架构（按钮**不在本进程发布**）：面板按钮 → POST /inject → 调
/grasp_hexapod/sim_inject 服务 → **sim_manual.py 节点**发布对应话题/应答
服务，让行为树一次只通过当前一小步。sim_manual 是唯一手动模拟源；面板
状态（开关/待确认模式名/装填/日志）由其 2Hz 发布的
/grasp_hexapod/sim_state 提供（5s 无帧显示"未连接"横幅）。
动作注册表（ACTIONS/GROUPS/PHASE_HINTS）从 sim_manual import（单一数据源）。

接口：
    GET  /           自包含 HTML 页面（1s 轮询 state.json，载荷未变不重渲染）
    GET  /state.json 最新快照 JSON（{tree_name,root_status,mission_status,
                    active_phase,active_feedback,nodes[],received,stale,waiting,
                    inject{connected,log[],hold{},mode_service{},gripper{}}}）
    POST /inject     {"action":"<id>"} 转发给 sim_manual（ACTIONS 注册表）

手动注入面（实机无反馈时的"保树运行"按钮，全部经 sim_manual 执行）：
    ▶通过当前步骤        按 active_phase 自动识别当前卡点，一键只放行这一小步
    任务命令/下放/回收   -> /lora/command（CMD,HEX,<OP>,MANUAL，锁存语义）
    编码器三态           -> /grasp_hexapod/encoder_state（落地/未落地/故障）
    传感器健康/分路异常   -> /grasp_hexapod/sensor_health
    RTK 协方差           -> /fix（良好 0.01 / 超限 9.0）
    遥控测试链           -> /grasp_hexapod/remote_cmd
    持续保持（开关）      -> 5Hz 重发 健康帧+编码器normal+/fix
                            （落地位=否：落地门由「确认落地」独立放行）
    手动模式服务（开关）  -> sim_manual 挂载 /grasp_hexapod/switch_mode：
                            即时 success / 单步确认（按钮实时显示待确认模式名）
                            / 装填一次性失败（任意下一次或指定模式）
    手动夹爪服务（开关）  -> sim_manual 挂载 /grasp_hexapod/gripper_act：
                            open/clamp 即时 success / 单步确认 / 装填一次性失败
                            （两服务与 sim_feedback/实机提供方互斥，默认关）

性能设计：/state.json 载荷仅在快照/注入视图变化或 stale 翻转时序列化一次，
HTTP keep-alive 复用连接，页面 1s 轮询 + 载荷字符串比较未变即跳过 DOM 更新，
RUNNING 呼吸灯只动画 opacity；注入动作默认一次性发布单帧。

用法：
    rosrun grasp_hexapod_bt bt_dashboard.py                 # 默认 0.0.0.0:8080
    rosrun grasp_hexapod_bt bt_dashboard.py _port:=9000     # 换端口
    配套：rosrun grasp_hexapod_bt sim_manual.py             # 按钮的执行端
    python3 bt_dashboard.py --selftest                      # 离线自检（不依赖 ROS）
"""

import argparse
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sim_manual
from sim_manual import (ACTIONS, GROUPS, PHASE_HINTS, SENSOR_NAMES,
                        SIM_INJECT_SERVICE, SIM_STATE_TOPIC)

TOPIC = "/grasp_hexapod/bt_state"


# ---------------------------------------------------------------------------
# 页面（自包含，无模板占位；数据由 JS 轮询 /state.json 渲染，
# 注入面板按钮由 ACTIONS/PHASE_HINTS 注入的 JSON 生成）
# ---------------------------------------------------------------------------
_PAGE_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Hexapod 行为树实时看板</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin:0; background:#0f172a; color:#e2e8f0;
         font: 14px/1.5 "SF Mono", Consolas, "Noto Sans Mono CJK SC", monospace; }
  header { padding:10px 16px; background:#1e293b; border-bottom:2px solid #334155;
           display:flex; flex-wrap:wrap; gap:10px; align-items:center; }
  header .title { font-weight:700; font-size:16px; margin-right:auto; }
  .chip { padding:3px 10px; border-radius:12px; font-size:13px; font-weight:700;
          border:1px solid transparent; white-space:nowrap; }
  .chip small { font-weight:400; opacity:.85; }
  .st  { display:inline-block; width:9px; height:9px; border-radius:50%;
         margin-right:5px; vertical-align:middle; }
  #staleBar { display:none; background:#7c2d12; color:#fed7aa; padding:4px 16px; font-size:12px; }
  #simBar { display:none; background:#7c2d12; color:#fed7aa; padding:4px 16px; font-size:12px; }
  .legend { display:flex; gap:14px; padding:6px 16px; background:#0b1120;
            border-bottom:1px solid #334155; font-size:12px; color:#94a3b8; }
  .layout { display:flex; align-items:stretch; }
  #left { flex:1 1 auto; min-width:0; }
  #phaseBox { padding:8px 16px; background:#1e293b; border-bottom:1px solid #334155; }
  #phaseBox .label { color:#94a3b8; font-size:12px; }
  #phaseName { font-size:18px; font-weight:700; margin:2px 0; }
  #phaseFb   { color:#cbd5e1; font-size:13px; min-height:18px; }
  #waiting { display:none; padding:30px; text-align:center; color:#94a3b8; font-size:15px; }

  /* ---- 树状图：嵌套列表 + 连接线 ---- */
  #treeWrap { padding:12px 10px 60px; overflow:auto; }
  ul.tree, ul.tree ul { list-style:none; margin:0; padding:0 0 0 20px; position:relative; }
  ul.tree { padding-left:4px; }
  ul.tree li { position:relative; padding:3px 0 3px 16px; }
  ul.tree li::before { content:""; position:absolute; left:0; top:0; bottom:0;
                       width:1px; background:#263449; }
  ul.tree li::after  { content:""; position:absolute; left:0; top:16px;
                       width:12px; height:1px; background:#263449; }
  ul.tree li:last-child::before { bottom:auto; height:16px; }
  .node { display:inline-block; max-width:640px; padding:3px 10px 4px 8px;
          border-radius:6px; border:1px solid #263449; border-left:3px solid #64748b;
          background:#131c30; }
  .node .nm { font-weight:600; }
  .node .fb { color:#94a3b8; font-size:12px; margin-top:1px; white-space:normal; }
  .node.r-running { border-left-color:#3b82f6; background:rgba(59,130,246,.10); }
  .node.r-success { border-left-color:#22c55e; }
  .node.r-failure { border-left-color:#ef4444; background:rgba(239,68,68,.08); }
  .node.r-invalid { border-left-color:#475569; opacity:.55; }
  .node.onpath { border-color:#3b4f77; }
  .node.dimmed { opacity:.5; }
  .node .dot { display:inline-block; width:9px; height:9px; border-radius:50%;
               margin-right:6px; vertical-align:middle; }
  .node .dot.pulse { animation: pulse 1.6s ease-in-out infinite; }
  @keyframes pulse { 0%,100% { opacity:1; } 50% { opacity:.35; } }
  @media (prefers-reduced-motion: reduce) { .node .dot.pulse { animation:none; } }
  .branch-mark { color:#64748b; font-size:11px; margin-right:3px; }

  /* ---- 注入面板 ---- */
  #panel { flex:0 0 330px; border-left:1px solid #334155; background:#0b1120;
           padding:10px 12px 40px; overflow-y:auto; max-height:calc(100vh - 96px); }
  @media (max-width:860px) {
    .layout { flex-direction:column; }
    #panel { flex:none; max-height:none; border-left:none; border-top:1px solid #334155; }
  }
  #panel h2 { margin:2px 0 8px; font-size:15px; }
  #panel h2 small { color:#94a3b8; font-weight:400; font-size:12px; }
  #injTip { display:none; margin:0 0 8px; padding:6px 8px; border-radius:6px;
            background:rgba(245,158,11,.12); border:1px solid #92400e;
            color:#fcd34d; font-size:12px; }
  #injTip.pend { background:rgba(59,130,246,.14); border-color:#1d4ed8;
                 color:#bfdbfe; }
  .grp { margin-bottom:10px; }
  .grp-title { color:#94a3b8; font-size:12px; margin-bottom:4px;
               border-bottom:1px dashed #263449; padding-bottom:2px; }
  .btns { display:flex; flex-wrap:wrap; gap:6px; }
  button.inj { padding:5px 10px; border-radius:6px; cursor:pointer;
               background:#1e293b; color:#e2e8f0; border:1px solid #334155;
               font:inherit; font-size:13px; }
  button.inj:hover { border-color:#60a5fa; }
  button.inj.suggest { border-color:#f59e0b; box-shadow:0 0 0 1px #f59e0b; }
  button.inj.on { background:#166534; border-color:#22c55e; }
  button.inj.warn { border-color:#7f1d1d; color:#fca5a5; }
  button.inj.warn:hover { border-color:#ef4444; }
  button.inj.hi { background:#052e16; border-color:#22c55e; color:#bbf7d0;
                  font-weight:700; }
  button.inj.hi:hover { box-shadow:0 0 0 1px #22c55e; }
  button.inj:disabled { opacity:.45; cursor:not-allowed; }
  .note { color:#64748b; font-size:11px; margin:2px 0 4px; }
  #injLog { margin-top:10px; border-top:1px solid #263449; padding-top:6px; }
  #injLog .t { color:#64748b; margin-right:6px; }
  #injLog .bad { color:#f87171; }
  #panel.off .btns { opacity:.45; }
  #toast { position:fixed; top:12px; right:12px; padding:8px 14px; border-radius:8px;
           background:#14532d; border:1px solid #22c55e; color:#dcfce7; font-size:13px;
           opacity:0; transition:opacity .25s; pointer-events:none; max-width:60vw; }
  #toast.show { opacity:1; }
  #toast.bad { background:#7f1d1d; border-color:#ef4444; color:#fee2e2; }
  footer { position:fixed; bottom:0; right:8px; color:#475569; font-size:11px;
           background:#0f172a; padding:0 4px; }
</style>
</head>
<body>
<header>
  <span class="title">Hexapod 行为树 · 实时状态</span>
  <span id="chipRoot" class="chip">根状态 —</span>
  <span id="chipMission" class="chip">任务结果 —</span>
  <span id="chipTime" class="chip"><small>—</small></span>
</header>
<div id="staleBar">⚠ 数据源超过 3 秒未更新（bt_state 已停止？运行器是否仍在运行）</div>
<div id="simBar">⚠ 模拟节点未连接（按钮不可用）——请启动：rosrun grasp_hexapod_bt sim_manual.py</div>
<div class="legend">
  <span><span class="st" style="background:#3b82f6"></span>RUNNING 执行中</span>
  <span><span class="st" style="background:#22c55e"></span>SUCCESS 已完成</span>
  <span><span class="st" style="background:#ef4444"></span>FAILURE 失败</span>
  <span><span class="st" style="background:#64748b"></span>INVALID 未访问</span>
</div>
<div class="layout">
  <div id="left">
    <div id="phaseBox">
      <div class="label">当前运行阶段</div>
      <div id="phaseName">等待数据…</div>
      <div id="phaseFb"></div>
    </div>
    <div id="waiting">等待 /grasp_hexapod/bt_state……<br>
      请确认行为树运行器已启动：<b>run_real_bt.py</b>（实机）或 <b>bt_mock_world.py</b>（联调模拟）。</div>
    <div id="treeWrap"></div>
  </div>
  <aside id="panel">
    <h2>反馈注入 <small>经 sim_manual 节点放行 · 一次一小步</small></h2>
    <div id="injTip"></div>
    <div id="injGroups"></div>
    <div id="injLog"></div>
  </aside>
</div>
<div id="toast"></div>
<footer>自动刷新 1s · bt_dashboard</footer>
<script>
"use strict";
var COLORS = { RUNNING:"#3b82f6", SUCCESS:"#22c55e", FAILURE:"#ef4444", INVALID:"#64748b" };
var MARKS  = { RUNNING:"\\u25cf", SUCCESS:"\\u2713", FAILURE:"\\u2717", INVALID:"\\u00b7" };
var ACTIONS = __ACTIONS_JSON__;
var GROUPS = __GROUPS_JSON__;
var HINTS = __HINTS_JSON__;
var BY_ID = {};
ACTIONS.forEach(function(a){ BY_ID[a.id] = a; });

function esc(s){ return (s==null?"":String(s)).replace(/[&<>"']/g,
  function(c){ return {"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]; }); }

function setChip(el, label, value){
  el.innerHTML = "<span class='st' style='background:"+(COLORS[value]||"#64748b")+
    "'></span>" + label + " " + esc(value||"—");
}

/* ---- 树：前序遍历 + depth 重建父子层级 ---- */
function buildHierarchy(nodes){
  var root = { n: nodes[0], children: [] }, stack = [root];
  for (var i = 1; i < nodes.length; i++){
    var nd = nodes[i];
    while (stack.length > 1 && stack[stack.length-1].n.depth >= nd.depth) stack.pop();
    var item = { n: nd, children: [] };
    stack[stack.length-1].children.push(item);
    stack.push(item);
  }
  return root;
}
function markPath(item, activeName){          // 标注活跃路径（含自身/后代命中）
  var hit = (item.n.status === "RUNNING" && item.n.name === activeName);
  item.children.forEach(function(c){ if (markPath(c, activeName)) hit = true; });
  item.onpath = hit;
  return hit;
}
function nodeHtml(item, hasActive){
  var n = item.n;
  var cls = "node r-" + String(n.status||"INVALID").toLowerCase();
  if (hasActive){ cls += item.onpath ? " onpath" : " dimmed"; }
  var mark = item.children.length
    ? "<span class='branch-mark'>\\u25be</span>" : "";
  var dot = "<span class='dot" + (n.status==="RUNNING" ? " pulse" : "") +
    "' style='background:" + (COLORS[n.status]||"#64748b") + "'></span>";
  var fb = n.feedback ? "<div class='fb'>" + esc(n.feedback) + "</div>" : "";
  var html = "<li><div class='" + cls + "'>" + mark + dot +
    "<span class='nm'>" + esc(n.name) + "</span>" + fb + "</div>";
  if (item.children.length){
    html += "<ul>" + item.children.map(function(c){ return nodeHtml(c, hasActive); }).join("") + "</ul>";
  }
  return html + "</li>";
}
function renderTree(nodes, activeName){
  var w = document.getElementById("treeWrap");
  if (!nodes || !nodes.length){ w.innerHTML = ""; return; }
  var root = buildHierarchy(nodes);
  var hasActive = nodes.some(function(n){ return n.status === "RUNNING"; });
  if (hasActive) markPath(root, activeName);
  w.innerHTML = "<ul class='tree'>" + nodeHtml(root, hasActive) + "</ul>";
}

/* ---- 注入面板 ---- */
function buildPanel(){
  var host = document.getElementById("injGroups"), html = "";
  GROUPS.forEach(function(g){
    var items = ACTIONS.filter(function(a){ return a.group === g; });
    if (!items.length) return;
    html += "<div class='grp'><div class='grp-title'>" + esc(g) + "</div><div class='btns'>";
    items.forEach(function(a){
      html += "<button class='inj" + (a.cls ? " " + a.cls : "") +
        "' data-act='" + a.id + "' title='" + esc(a.note||"") + "'>" +
        esc(a.label) + "</button>";
    });
    html += "</div>";
    items.forEach(function(a){
      html += "<div class='note' data-note='" + a.id + "'>" + esc(a.note||"") + "</div>";
    });
    html += "</div>";
  });
  host.innerHTML = html;
  host.addEventListener("click", function(ev){
    var b = ev.target.closest("button.inj");
    if (!b || b.disabled) return;
    inject(b.dataset.act, b);
  });
}
var BINDS = { hold: "hold_toggle", mode_service: "mode_service_toggle",
              gripper: "gripper_service_toggle" };
function relabelConfirm(d){
  /* 待确认目标上屏：确认按钮实时显示阻塞中的模式名/夹爪动作 */
  var inj = d.inject || {};
  var mp = inj.mode_service && inj.mode_service.pending;
  var okB = document.querySelector("button[data-act=mode_confirm_ok]");
  var failB = document.querySelector("button[data-act=mode_confirm_fail]");
  if (okB) okB.textContent = (mp && mp.mode)
    ? "确认 " + mp.mode + " 成功" : BY_ID.mode_confirm_ok.label;
  if (failB) failB.textContent = (mp && mp.mode)
    ? "确认 " + mp.mode + " 失败" : BY_ID.mode_confirm_fail.label;
  var gp = inj.gripper && inj.gripper.pending;
  var gok = document.querySelector("button[data-act=gripper_confirm_ok]");
  var gfail = document.querySelector("button[data-act=gripper_confirm_fail]");
  if (gok) gok.textContent = (gp && gp.action)
    ? "确认 " + gp.action + " 成功" : BY_ID.gripper_confirm_ok.label;
  if (gfail) gfail.textContent = (gp && gp.action)
    ? "确认 " + gp.action + " 失败" : BY_ID.gripper_confirm_fail.label;
}
function updateToggles(d){
  var inj = d.inject || {};
  Object.keys(BINDS).forEach(function(key){
    var btn = document.querySelector("button[data-act=" + BINDS[key] + "]");
    if (!btn) return;
    var st = inj[key] || {};
    btn.classList.toggle("on", !!st.on);
    btn.textContent = BY_ID[btn.dataset.act].label + (st.on ? " · 开" : " · 关");
    var noteEl = document.querySelector("[data-note=" + btn.dataset.act + "]");
    if (noteEl){
      var parts = [];
      if (key === "hold" && st.on && st.since){
        parts.push("已运行 " + Math.max(0, Math.round(
          Date.now()/1000 - st.since)) + "s");
      }
      if (key === "mode_service" && !st.on && st.occupied){
        parts.push("switch_mode 已被其他节点占用");
      }
      if (key === "gripper" && !st.on && st.occupied){
        parts.push("gripper_act 已被其他节点占用");
      }
      if (key === "mode_service" && st.fail_armed){
        parts.push("【已装填:下次模式失败】");
      }
      if (key === "mode_service" && st.fail_armed_modes && st.fail_armed_modes.length){
        parts.push("【已装填:" + st.fail_armed_modes.join("/") + " 失败】");
      }
      if (key === "gripper" && st.fail_armed){
        var armed = ["open", "clamp"].filter(function(k){
          return st.fail_armed[k]; });
        if (armed.length) parts.push("【已装填:下次 " + armed.join("/") + " 失败】");
      }
      noteEl.textContent = (BY_ID[btn.dataset.act].note || "") +
        (parts.length ? "（" + parts.join("，") + "）" : "");
    }
  });
  var ms = inj.mode_service || {}, gr = inj.gripper || {};
  var stepBtn = document.querySelector("button[data-act=mode_step_toggle]");
  if (stepBtn){
    stepBtn.classList.toggle("on", !!ms.step);
    stepBtn.textContent = BY_ID.mode_step_toggle.label +
      (ms.step ? " · 开" : " · 关");
  }
  var gstepBtn = document.querySelector("button[data-act=gripper_step_toggle]");
  if (gstepBtn){
    gstepBtn.classList.toggle("on", !!gr.step);
    gstepBtn.textContent = BY_ID.gripper_step_toggle.label +
      (gr.step ? " · 开" : " · 关");
  }
  relabelConfirm(d);
}
var pendingSince = 0, pendingMode = "", pendingCount = 1;
var gripSince = 0, gripAction = "";
function setPendText(p){
  pendingSince = p.since || Date.now()/1000;
  pendingMode = p.mode;
  pendingCount = p.count || 1;
  var waited = Math.max(0, Math.round(Date.now()/1000 - pendingSince));
  var cnt = pendingCount > 1 ? "（共 " + pendingCount + " 个等待）" : "";
  document.getElementById("injTip").textContent =
    "\\u23f8 单步等待确认 switch_mode(" + pendingMode + ") 已等 " + waited +
    "s" + cnt + " \\u2192 点「确认模式成功 / 失败」放行";
}
function setGripText(p){
  gripSince = p.since || Date.now()/1000;
  gripAction = p.action;
  var waited = Math.max(0, Math.round(Date.now()/1000 - gripSince));
  document.getElementById("injTip").textContent =
    "\\u23f8 单步等待确认 gripper(" + gripAction + ") 已等 " + waited +
    "s \\u2192 点「确认夹爪成功 / 失败」放行";
}
setInterval(function(){          // 仅单步等待期间每秒刷新计时文本
  var tip = document.getElementById("injTip");
  if (tip.className.indexOf("pend") < 0) return;
  if (pendingSince) setPendText({mode: pendingMode, since: pendingSince,
                                 count: pendingCount});
  else if (gripSince) setGripText({action: gripAction, since: gripSince});
}, 1000);
function updateSuggest(d){
  var tip = document.getElementById("injTip");
  document.querySelectorAll("button.inj.suggest").forEach(function(b){
    b.classList.remove("suggest"); });
  var pend = d.inject && d.inject.mode_service && d.inject.mode_service.pending;
  var gpend = d.inject && d.inject.gripper && d.inject.gripper.pending;
  if (!(pend && pend.mode)) pendingSince = 0;
  if (!(gpend && gpend.action)) gripSince = 0;
  if (pend && pend.mode){                   // 单步等待优先于阶段建议
    tip.className = "pend";
    tip.style.display = "block";
    setPendText(pend);
    ["mode_confirm_ok", "mode_confirm_fail"].forEach(function(id){
      var b = document.querySelector("button[data-act=" + id + "]");
      if (b) b.classList.add("suggest"); });
    return;
  }
  if (gpend && gpend.action){               // 夹爪单步等待
    tip.className = "pend";
    tip.style.display = "block";
    setGripText(gpend);
    ["gripper_confirm_ok", "gripper_confirm_fail"].forEach(function(id){
      var b = document.querySelector("button[data-act=" + id + "]");
      if (b) b.classList.add("suggest"); });
    return;
  }
  tip.className = "";
  var hint = null;
  for (var i = 0; i < HINTS.length; i++){
    if (d.active_phase && d.active_phase.indexOf(HINTS[i].match) === 0){ hint = HINTS[i]; break; }
  }
  if (!hint && (d.tree_name||"").indexOf("遥控") >= 0){
    hint = { actions: ACTIONS.filter(function(a){ return a.group === "⑦ 遥控测试链"; })
              .map(function(a){ return a.id; }),
             tip: "遥控测试链运行中 → 点对应 remote 按钮切换模式" };
  }
  if (hint){
    tip.style.display = "block";
    tip.textContent = "▸ " + hint.tip;
    hint.actions.forEach(function(id){
      var b = document.querySelector("button[data-act=" + id + "]");
      if (b) b.classList.add("suggest"); });
  } else {
    tip.style.display = "none";
  }
}
function renderLog(log){
  var host = document.getElementById("injLog");
  if (!log || !log.length){ host.innerHTML = ""; return; }
  host.innerHTML = log.slice(-8).map(function(e){
    var t = e.t ? new Date(e.t*1000).toLocaleTimeString() : "--:--:--";
    return "<div><span class='t'>" + t + "</span>" + (e.ok ? "\\u2713 " : "\\u2717 ") +
      esc(e.action) + " <span" + (e.ok ? "" : " class='bad'") + ">" + esc(e.msg||"") +
      "</span></div>";
  }).join("");
}
var toastTimer = null;
function toast(msg, ok){
  var t = document.getElementById("toast");
  t.textContent = msg;
  t.className = "show" + (ok ? "" : " bad");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(function(){ t.className = ""; }, 2600);
}
function inject(act, btn){
  btn.disabled = true;                       // LoRa 命令是锁存语义，防双击重复注入
  fetch("inject", { method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ action: act }) })
    .then(function(r){ return r.json(); })
    .then(function(res){
      btn.disabled = false;
      toast(res.ok ? "\\u2714 " + res.msg : "\\u2718 " + res.msg, res.ok);
      poll();
    })
    .catch(function(){ btn.disabled = false; toast("注入请求失败", false); });
}

/* ---- 主渲染（仅在载荷变化时被调用） ---- */
function render(d){
  document.getElementById("waiting").style.display = d.waiting ? "block" : "none";
  var pendNow = d.inject && d.inject.mode_service && d.inject.mode_service.pending;
  document.getElementById("staleBar").style.display =
    (d.stale && !pendNow) ? "block" : "none";   // 单步阻塞时树暂停 tick，不算数据断流
  setChip(document.getElementById("chipRoot"), "树状态", d.root_status);
  setChip(document.getElementById("chipMission"), "任务结果", d.mission_status);
  var t = d.received ? new Date(d.received*1000).toLocaleTimeString() : "—";
  document.getElementById("chipTime").innerHTML = "<small>" + t + "</small>";
  document.getElementById("phaseName").textContent =
    d.active_phase || (d.waiting ? "等待数据…" : "待命 / 无运行节点");
  document.getElementById("phaseFb").textContent = d.active_feedback || "";
  renderTree(d.nodes || [], d.active_phase);
  var panel = document.getElementById("panel");
  var inj = d.inject || {};
  var connected = inj.connected === true;
  var avail = connected && inj.available !== false;
  document.getElementById("simBar").style.display =
    connected ? "none" : "block";
  panel.classList.toggle("off", !avail);
  document.querySelectorAll("button.inj").forEach(function(b){
    if (!b.classList.contains("on")) b.disabled = !avail; });
  updateToggles(d);
  updateSuggest(d);
  renderLog(d.inject && d.inject.log);
}

/* ---- 1s 轮询：防重叠 + 载荷未变跳过全部 DOM 更新 ---- */
var inflight = false, lastPayload = "";
function poll(){
  if (inflight) return;
  inflight = true;
  fetch("state.json", { cache: "no-store" })
    .then(function(r){ return r.text(); })
    .then(function(txt){
      inflight = false;
      if (txt === lastPayload) return;       // 无变化零开销
      lastPayload = txt;
      render(JSON.parse(txt));
    })
    .catch(function(){ inflight = false; });
}
buildPanel();
setInterval(poll, 1000);
poll();
</script>
</body>
</html>
"""

PAGE = (_PAGE_TEMPLATE
        .replace("__ACTIONS_JSON__", json.dumps(
            [dict(id=k, **v) for k, v in ACTIONS.items()], ensure_ascii=False))
        .replace("__GROUPS_JSON__", json.dumps(GROUPS, ensure_ascii=False))
        .replace("__HINTS_JSON__", json.dumps(PHASE_HINTS, ensure_ascii=False)))


# ---------------------------------------------------------------------------
# 看板 → sim_manual 转发链（run() 内构造；离线用 FakeSimLink）
# ---------------------------------------------------------------------------
class SimLink:
    """按钮动作 → /grasp_hexapod/sim_inject 服务（sim_manual 执行发布）。

    本进程不发布任何模拟话题；面板状态来自 sim_manual 2Hz 发布的
    /grasp_hexapod/sim_state（5s 无帧视为未连接，横幅提示+按钮禁用）。
    服务调用在后台线程执行（3s 上限），避免节点挂起拖死 HTTP 线程。
    """

    CALL_TIMEOUT_S = 3.0
    STATE_STALE_S = 5.0

    def __init__(self):
        self._lock = threading.Lock()
        self._raw = None            # 最近一帧 sim_state（dict）
        self._state_t = 0.0
        self._view = None
        self._proxy = None

    # ---- sim_state 订阅回调 ----
    def on_sim_state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._raw = data
            self._state_t = time.time()
            self._view = None          # 新帧 → 视图缓存失效

    # ---- POST /inject 处理 ----
    def inject(self, action):
        import rospy
        if not sim_manual.known_action(action):
            return (False, "未知动作: {}".format(action))
        try:
            rospy.wait_for_service(SIM_INJECT_SERVICE,
                                   timeout=self.CALL_TIMEOUT_S)
        except Exception:  # noqa: BLE001
            self._proxy = None
            return (False, "sim_manual 未运行（先启动: "
                           "rosrun grasp_hexapod_bt sim_manual.py）")
        box = {}

        def _call():
            try:
                from grasp_hexapod_msgs.srv import SimInject
                proxy = self._proxy
                if proxy is None:
                    proxy = rospy.ServiceProxy(SIM_INJECT_SERVICE, SimInject)
                    self._proxy = proxy
                resp = proxy(action)
                box["res"] = (bool(resp.success), resp.message or "")
            except Exception as exc:  # noqa: BLE001
                self._proxy = None
                box["res"] = (False, "调用失败: {}".format(exc))

        th = threading.Thread(target=_call, daemon=True)
        th.start()
        th.join(self.CALL_TIMEOUT_S)
        if "res" not in box:
            return (False, "sim_manual 响应超时（进程挂起？）")
        return box["res"]

    # ---- 面板状态视图（内容未变保持对象身份以命中载荷缓存） ----
    def view(self):
        with self._lock:
            raw = self._raw
            state_t = self._state_t
        if raw is None:
            view = {"available": False, "connected": False}
        else:
            view = dict(raw)
            view["available"] = True
            view["connected"] = time.time() - state_t <= self.STATE_STALE_S
        if view != self._view:
            self._view = view
        return self._view


class FakeSimLink:
    """离线假转发：ManualState 模拟开关/装填/单步状态（不依赖 ROS、不发布）。

    与 SimLink 同接口（inject/view），供 --selftest 走完整 HTTP 往返。
    """

    def __init__(self):
        self.core = sim_manual.ManualState("recover")
        self._mode_svc = False
        self._gripper_svc = False
        self._view = None

    def inject(self, action):
        core = self.core
        if not sim_manual.known_action(action):
            return core.log_result(action, False, "未知动作: {}".format(action))
        if action == "hold_toggle":
            core.hold_on = not core.hold_on
            return core.log_result(
                action, True, "持续保持已{}（离线模拟）".format(
                    "开启" if core.hold_on else "关闭"))
        if action == "mode_service_toggle":
            self._mode_svc = not self._mode_svc
            return core.log_result(
                action, True, "手动模式服务已{}（离线模拟）".format(
                    "开启" if self._mode_svc else "关闭"))
        if action == "gripper_service_toggle":
            self._gripper_svc = not self._gripper_svc
            return core.log_result(
                action, True, "手动夹爪服务已{}（离线模拟）".format(
                    "开启" if self._gripper_svc else "关闭"))
        if action == "mode_step_toggle":
            core.mode_step = not core.mode_step
            return core.log_result(
                action, True, "模式单步确认已{}（离线模拟）".format(
                    "开启" if core.mode_step else "关闭"))
        if action == "gripper_step_toggle":
            core.gripper_step = not core.gripper_step
            return core.log_result(
                action, True, "夹爪单步确认已{}（离线模拟）".format(
                    "开启" if core.gripper_step else "关闭"))
        if action in ("mode_confirm_ok", "mode_confirm_fail"):
            ok, msg = core.confirm_mode(action == "mode_confirm_ok")
            return core.log_result(action, ok, msg)
        if action in ("gripper_confirm_ok", "gripper_confirm_fail"):
            ok, msg = core.confirm_gripper(action == "gripper_confirm_ok")
            return core.log_result(action, ok, msg)
        if action == "mode_fail_next":
            ok, msg = core.arm_mode_fail(None)
            return core.log_result(action, ok, msg)
        if action.startswith("mode_fail_"):
            ok, msg = core.arm_mode_fail(action[len("mode_fail_"):])
            return core.log_result(action, ok, msg)
        if action in ("gripper_fail_open", "gripper_fail_clamp"):
            ok, msg = core.arm_gripper_fail(action[len("gripper_fail_"):])
            return core.log_result(action, ok, msg)
        return core.log_result(action, True, "离线模拟注入: {}".format(action))

    def view(self):
        view = dict(self.core.view())
        mode_svc = dict(view["mode_service"])
        mode_svc["on"], mode_svc["occupied"] = self._mode_svc, False
        gripper_svc = dict(view["gripper"])
        gripper_svc["on"], gripper_svc["occupied"] = self._gripper_svc, False
        view["mode_service"], view["gripper"] = mode_svc, gripper_svc
        view["available"] = True
        view["connected"] = True
        if view != self._view:
            self._view = view
        return self._view


# ---------------------------------------------------------------------------
# 最新快照容器（订阅回调写、HTTP 读，跨线程）。
# 载荷缓存：仅在 快照/注入视图 变化或 stale 翻转时重新序列化。
# ---------------------------------------------------------------------------
class DashboardState:
    _WAITING = {"tree_name": "", "root_status": "", "mission_status": "",
                "active_phase": "", "active_feedback": "", "nodes": []}

    def __init__(self):
        self._lock = threading.Lock()
        self.snapshot = None
        self.stamp = 0.0
        self._cache = None
        self._cache_key = None

    def update(self, snapshot):
        with self._lock:
            self.snapshot = snapshot
            self.stamp = time.time()

    def json_payload(self, injector=None):
        view = injector.view() if injector is not None else {}
        with self._lock:
            stale = (self.snapshot is not None
                     and time.time() - self.stamp > 3.0)
            key = (id(self.snapshot), id(view), stale)
            if self._cache_key != key:
                if self.snapshot is not None:
                    data = dict(self.snapshot)
                    data["received"] = self.stamp
                    data["waiting"] = False
                else:
                    data = dict(self._WAITING)
                    data["received"] = 0
                    data["waiting"] = True
                data["stale"] = stale
                data["inject"] = view
                self._cache = json.dumps(data, ensure_ascii=False)
                self._cache_key = key
            return self._cache


class Handler(BaseHTTPRequestHandler):
    state = DashboardState()     # run() 里覆盖，便于自检注入
    injector = None              # run()/selftest 注入
    protocol_version = "HTTP/1.1"   # keep-alive：轮询复用连接

    def _respond(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):  # noqa: BLE001
            pass

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._respond(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif path == "/state.json":
            self._respond(200, self.state.json_payload(self.injector).encode("utf-8"),
                          "application/json; charset=utf-8")
        else:
            self.send_error(404, "not found")

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path != "/inject":
            self.send_error(404, "not found")
            return
        try:
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
        except Exception:  # noqa: BLE001
            self._respond(400, json.dumps({"ok": False,
                                           "msg": "请求体不是合法 JSON"}).encode("utf-8"),
                          "application/json; charset=utf-8")
            return
        action = body.get("action", "")
        if self.injector is None:
            self._respond(503, json.dumps({"ok": False,
                                           "msg": "注入不可用（无注入器）"}).encode("utf-8"),
                          "application/json; charset=utf-8")
            return
        ok, msg = self.injector.inject(action)
        self._respond(200 if ok else 400,
                      json.dumps({"ok": ok, "msg": msg},
                                 ensure_ascii=False).encode("utf-8"),
                      "application/json; charset=utf-8")

    def log_message(self, *args):   # 静默访问日志（避免刷屏）
        pass


def main():
    parser = argparse.ArgumentParser(description="行为树 Web 实时看板（含手动反馈注入）")
    parser.add_argument("--selftest", action="store_true",
                        help="离线自检（不依赖 ROS）")
    args, _ = parser.parse_known_args()
    if args.selftest:
        selftest()
        return
    run()


def run():
    import rospy
    from std_msgs.msg import String
    from grasp_hexapod_msgs.msg import BtStateArray
    from bt_monitor import snapshot_from_msg   # 复用 msg→快照转换（无 ROS 顶层导入）

    rospy.init_node("bt_dashboard", anonymous=True)
    port = int(rospy.get_param("~port", 8080))
    host = rospy.get_param("~host", "0.0.0.0")

    state = DashboardState()
    simlink = SimLink()
    Handler.state = state
    Handler.injector = simlink

    def on_state(msg):
        state.update(snapshot_from_msg(msg))

    rospy.Subscriber(TOPIC, BtStateArray, on_state, queue_size=10)
    rospy.Subscriber(SIM_STATE_TOPIC, String, simlink.on_sim_state,
                     queue_size=5)
    try:
        httpd = ThreadingHTTPServer((host, port), Handler)
    except OSError as exc:
        rospy.logfatal("无法监听 %s:%d（%s）。多为残留的旧看板进程占用："
                       "ps aux | grep bt_dashboard 找到后 kill（挂起态需 kill -CONT/-9），"
                       "或 rosrun grasp_hexapod_bt bt_dashboard.py _port:=9000 换端口",
                       host, port, exc)
        sys.exit(1)
    rospy.loginfo("bt_dashboard 就绪：http://%s:%d （订阅 %s；按钮经 %s 转发到 "
                  "sim_manual，未连接时页面顶部有横幅提示）",
                  _lan_ip(), port, TOPIC, SIM_INJECT_SERVICE)
    # rospy 接管 SIGINT 后 Ctrl+C 不向主线程抛异常（serve_forever 收不到
    # KeyboardInterrupt，进程关不掉）。on_shutdown 钩子在主线程的信号处理
    # 里执行，httpd.shutdown() 又会等 serve_forever 返回（也占主线程）——
    # 直接调用必死锁；必须另起线程去停。SIGTERM 同样走 ROS 关闭流程。
    import signal
    rospy.on_shutdown(lambda: threading.Thread(
        target=httpd.shutdown, daemon=True).start())
    signal.signal(signal.SIGTERM,
                  lambda *_a: rospy.signal_shutdown("sigterm"))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        rospy.signal_shutdown("bt_dashboard exit")


def _lan_ip():
    import socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except Exception:  # noqa: BLE001
        return "0.0.0.0"


def selftest():
    """离线：注册表、转发链、HTTP 往返（GET/POST/缓存）、页面自包含。"""
    import urllib.request
    import hexapod_bt

    # 1. 空快照 → waiting JSON（inject 块缺省为空）
    Handler.state = DashboardState()
    Handler.injector = None
    payload = json.loads(Handler.state.json_payload())
    assert payload["waiting"] is True and payload["nodes"] == []
    assert payload["inject"] == {}
    print("[OK] 无数据 JSON（waiting）")

    # 2. 注册表一致性：页面嵌入全部动作 id；PHASE_HINTS 引用都存在；
    #    新控制点齐备（step_next / 指定模式失败 / 夹爪单步）
    assert "__ACTIONS_JSON__" not in PAGE and "__HINTS_JSON__" not in PAGE
    for aid in ACTIONS:
        assert '"{}"'.format(aid) in PAGE, aid
    for hint in PHASE_HINTS:
        for aid in hint["actions"]:
            assert aid in ACTIONS, aid
    for act in ACTIONS.values():
        assert act["group"] in GROUPS, act
    assert "step_next" in ACTIONS and ACTIONS["step_next"]["group"] == GROUPS[0]
    assert "mode_fail_climb" in ACTIONS and "gripper_step_toggle" in ACTIONS
    assert "simBar" in PAGE and "sim_manual" in PAGE
    print("[OK] ACTIONS/PHASE_HINTS 注册表一致（%d 个动作，含新控制点）"
          % len(ACTIONS))

    # 3. SimLink：无帧 → 未连接视图（横幅数据源）
    link = SimLink()
    view = link.view()
    assert view["available"] is False and view["connected"] is False

    class _Msg:
        pass
    msg = _Msg()
    msg.data = json.dumps({"available": True, "mission": "recover",
                           "log": [], "hold": {"on": False, "since": 0.0},
                           "mode_service": {"on": True, "occupied": False,
                                            "step": True, "pending": None},
                           "gripper": {"on": False, "occupied": False,
                                       "step": False, "pending": None}})
    link.on_sim_state(msg)
    view = link.view()
    assert view["available"] is True and view["connected"] is True
    bad = _Msg()
    bad.data = "{not json"
    link.on_sim_state(bad)                     # 坏帧忽略，视图不变
    assert link.view() is view                 # 内容未变保持对象身份（载荷缓存）
    print("[OK] SimLink sim_state 帧解析/坏帧忽略/未连接视图")

    # 4. FakeSimLink：一次性/开关/装填/单步/未知动作 + 视图
    fake = FakeSimLink()
    ok, _ = fake.inject("deploy")
    assert ok
    ok, _ = fake.inject("bogus")
    assert not ok
    fake.inject("hold_toggle")
    fake.inject("mode_service_toggle")
    view = fake.view()
    assert view["hold"]["on"] and view["mode_service"]["on"]
    assert view["connected"] is True and view["available"] is True
    assert [e["action"] for e in view["log"]] == [
        "deploy", "bogus", "hold_toggle", "mode_service_toggle"]
    assert fake.view() is view                 # 内容未变保持对象身份

    # 4b. 细化动作：编码器两态/分路异常/装填（含指定模式）/夹爪单步
    for act in ("landed", "not_landed", "task_bogus", "rtk_bad",
                "sensor_bad_imu", "sensor_bad_mono", "step_next",
                "mode_fail_climb", "mode_fail_tag_nav"):
        ok, _ = fake.inject(act)
        assert ok, act
    fake.inject("mode_fail_next")
    fake.inject("gripper_fail_open")
    fake.inject("gripper_service_toggle")
    fake.inject("mode_step_toggle")
    fake.inject("gripper_step_toggle")
    view = fake.view()
    assert view["mode_service"]["fail_armed"] is True
    assert sorted(view["mode_service"]["fail_armed_modes"]) == [
        "climb", "tag_nav"]
    assert view["mode_service"]["step"] is True
    assert view["mode_service"]["pending"] is None
    assert view["gripper"]["fail_armed"]["open"] is True
    assert view["gripper"]["fail_armed"]["clamp"] is False
    assert view["gripper"]["on"] is True
    assert view["gripper"]["step"] is True
    ok, msg = fake.inject("mode_confirm_ok")
    assert not ok and "无待确认" in msg
    ok, msg = fake.inject("gripper_confirm_ok")
    assert not ok and "无待确认" in msg
    assert set(a for a in ACTIONS if a.startswith("sensor_bad_")) == set(
        "sensor_bad_" + n for n in SENSOR_NAMES)
    print("[OK] FakeSimLink 动作/开关/装填（含指定模式）/夹爪单步/视图")

    # 5. HTTP 往返：GET 页面与 state.json、POST /inject 成败两路、坏 JSON
    Handler.state = DashboardState()
    Handler.injector = fake
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    port = httpd.server_address[1]
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        base = "http://127.0.0.1:{}".format(port)
        page = urllib.request.urlopen(base + "/", timeout=5).read().decode("utf-8")
        assert 'fetch("state.json"' in page.replace(" ", "")
        assert "POST" in page and "data-act" in page and "bt_state" in page
        assert '<meta charset="utf-8">' in page
        assert "simBar" in page and "relabelConfirm" in page

        st = json.loads(urllib.request.urlopen(base + "/state.json",
                                               timeout=5).read().decode("utf-8"))
        assert st["waiting"] is True and st["inject"]["connected"] is True

        req = urllib.request.Request(
            base + "/inject", data=json.dumps({"action": "landed"}).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST")
        resp = json.loads(urllib.request.urlopen(req, timeout=5).read().decode("utf-8"))
        assert resp["ok"] is True
        assert "landed" in [e["action"] for e in fake.view()["log"]]

        try:
            req = urllib.request.Request(
                base + "/inject", data=json.dumps({"action": "zzz"}).encode("utf-8"),
                headers={"Content-Type": "application/json"}, method="POST")
            urllib.request.urlopen(req, timeout=5)
            raise AssertionError("未知动作应返回 400")
        except urllib.error.HTTPError as err:       # noqa: F821（urllib.error 同步可用）
            assert err.code == 400
        assert fake.inject("zzz")[0] is False
        print("[OK] HTTP 往返（GET /state.json、POST /inject 成败两路）")
    finally:
        httpd.shutdown()
        httpd.server_close()

    # 6. 载荷缓存：快照/注入视图不变时返回同一字符串对象
    Handler.state = DashboardState()
    p1 = Handler.state.json_payload(fake)
    p2 = Handler.state.json_payload(fake)
    assert p1 is p2
    fake.inject("deploy")                            # 视图失效 → 载荷变化
    assert Handler.state.json_payload(fake) != p1
    print("[OK] 载荷缓存（未变零序列化）")

    # 7. 注入真实快照（等价收到一帧 BtStateArray）+ 任务失败回退语义
    ctx = hexapod_bt.FakeBridge(script={"mission": "recover"})
    tree = hexapod_bt.build_hexapod_tree(ctx)
    for _ in range(14):                      # ~7s：推进过落地确认
        hexapod_bt._tick(tree, ctx, 0.5)
    snap = hexapod_bt.snapshot_tree(tree, mission_status="")
    Handler.state.update(snap)
    payload = json.loads(Handler.state.json_payload())
    assert payload["waiting"] is False
    assert payload["root_status"] in ("RUNNING", "SUCCESS")
    assert payload["nodes"][0]["depth"] == 0
    assert any(n["status"] == "RUNNING" for n in payload["nodes"])
    assert payload["received"] > 0 and payload["stale"] is False
    print("[OK] 注入真实快照 JSON（nodes=%d, 根=%s, 阶段=%s）" % (
        len(payload["nodes"]), payload["root_status"], payload["active_phase"]))

    snap2 = dict(snap)
    snap2["mission_status"] = "FAILED"
    snap2["root_status"] = "SUCCESS"   # 根为失败回退 Selector，任务失败仍 SUCCESS
    Handler.state.update(snap2)
    payload2 = json.loads(Handler.state.json_payload())
    assert (payload2["mission_status"] == "FAILED"
            and payload2["root_status"] == "SUCCESS")
    print("[OK] 任务失败回退语义：mission_status=FAILED 与根状态分离")

    print("selftest 全部通过")


if __name__ == "__main__":
    main()
