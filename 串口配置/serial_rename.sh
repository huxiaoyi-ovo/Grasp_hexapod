#!/usr/bin/env bash
#===============================================================================
# 串口重命名脚本 (bash 版)
#-------------------------------------------------------------------------------
# 功能:
#   1. 读取指定串口(如 /dev/ttyUSB0)的 udev 设备信息
#   2. 根据配置生成 udev 规则,为串口创建固定的软链接名称
#   3. 可选: 固定串口物理位置(基于 USB 端口路径 KERNELS)
#   4. 可选: 同时设置串口权限(MODE,默认 0666,解决非 dialout 组用户无法打开的问题)
#   5. 在脚本同目录下查找规则文件:
#        - 存在规则文件 -> 在现有规则文件中新增一条规则
#        - 不存在      -> 新建规则文件并添加规则
#   6. 将规则文件复制到系统 udev 规则目录 /etc/udev/rules.d/
#   7. 重新载入规则: udevadm control --reload-rules
#   8. 刷新串口: udevadm trigger,并验证新名称是否生效
#
# 用法:
#   ./serial_rename.sh                          # 使用配置区中的设置
#   ./serial_rename.sh /dev/ttyUSB1 ttyRobot    # 命令行覆盖串口和名称
#   (写入系统规则目录需要 root,非 root 运行时脚本会自动用 sudo 重新执行)
#
# 说明:
#   udev 无法安全地直接修改内核串口节点的名字,业界标准做法是通过
#   udev 规则为设备创建稳定的软链接,即 /dev/<新名称> 会指向实际的
#   /dev/ttyUSBx 或 /dev/ttyACMx。重新插拔后新名称依然有效。
#   若配置了 RULE_MODE,规则会同时设置设备节点权限(如 MODE="0666"),
#   避免普通用户因不在 dialout 组而无法打开串口。
#===============================================================================

# ==============================================================================
# ===== 用户配置区:修改这里的配置后保存,再运行脚本即可 =====
# ==============================================================================

# 1. 当前插入的串口设备(要被重命名的设备)
#    留空 "" 时,脚本会列出检测到的串口让你交互选择
CURRENT_PORT="/dev/ttyACM3"

# 2. 修改后的串口名称(软链接名,不带 /dev/ 前缀)
NEW_NAME="RTK"

# 3. 串口设备权限 (udev MODE 属性)
#    留空 "" 表示不改权限; 常用值:
#      0666 -> 所有用户可读写(最常见的免权限方案)
#      0660 -> root:dialout 组可读写(需将用户加入 dialout 组)
#    仅对脚本创建的软链接所指向的 tty 设备节点生效
RULE_MODE="0666"

# 4. 是否同时固定串口位置
#    1 -> 规则中加入物理端口路径(KERNELS),同一型号多个设备时靠插口位置区分
#    0 -> 仅按 VID/PID/序列号匹配,插在哪个 USB 口上名称都一样
FIX_POSITION=1

# 5. 是否在写入前询问确认 (1=询问, 0=直接执行)
ASK_CONFIRM=1

# 6. 规则文件相关配置
RULES_FILE_NAME="99-serial-rename.rules"   # 新建规则文件时使用的文件名(位于脚本同目录)
RULES_DIR="/etc/udev/rules.d"              # 系统 udev 规则目录

# ==============================================================================
# ===== 用户配置区结束,一般无需修改以下内容 =====
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- 输出辅助 ----
# 提示消息统一输出到 stderr,stdout 只留给函数返回值(如规则文件路径)
log()  { printf '\033[1;34m[信息]\033[0m %s\n' "$*" >&2; }
ok()   { printf '\033[1;32m[完成]\033[0m %s\n' "$*" >&2; }
warn() { printf '\033[1;33m[警告]\033[0m %s\n' "$*" >&2; }
err()  { printf '\033[1;31m[错误]\033[0m %s\n' "$*" >&2; }
step() { printf '\033[1;36m[步骤]\033[0m %s\n' "$*" >&2; }

# ---- 非 root 时自动用 sudo 重新执行 ----
ensure_root() {
    if [ "$(id -u)" -eq 0 ]; then
        return 0
    fi
    log "写入系统规则目录需要 root 权限,正在用 sudo 重新执行..."
    exec sudo bash "${BASH_SOURCE[0]}" "$@"
}

# ---- 读取串口 udev 信息 ----
# 结果写入全局变量: VID PID SERIAL POS
# 返回 0 成功, 1 失败
get_port_info() {
    local port="$1" out kv AWK_PROG
    VID=""; PID=""; SERIAL=""; POS=""

    command -v udevadm >/dev/null 2>&1 || { err "未找到 udevadm,请确认系统使用 udev"; return 1; }

    out="$(udevadm info -a -n "$port" 2>/dev/null)" || { err "udevadm 读取 $port 信息失败"; return 1; }

    # 按 "looking at" 分段解析:
    # 找出 KERNELS 形如 "1-1.3" / "3-2.1.4"(USB 设备层,不含 ":1.0" 接口后缀)的第一段,
    # 取其 idVendor / idProduct / serial。
    AWK_PROG="$(cat <<'AWK'
BEGIN { found = 0; kern = ""; vid = ""; pid = ""; serial = "" }
function flush() {
    if (!found && kern ~ /^[0-9]+-[0-9]+(\.[0-9]+)*$/) {
        print "VID=" vid
        print "PID=" pid
        print "SERIAL=" serial
        print "POS=" kern
        found = 1
    }
}
/looking at/ { flush(); kern = ""; vid = ""; pid = ""; serial = ""; next }
{
    if (match($0, /KERNELS=="[^"]*"/)) {
        t = substr($0, RSTART, RLENGTH)
        sub(/^[^=]*==/, "", t); gsub(/"/, "", t)
        kern = t
    }
    if (match($0, /ATTRS\{idVendor\}=="[^"]*"/)) {
        t = substr($0, RSTART, RLENGTH)
        sub(/^[^=]*==/, "", t); gsub(/"/, "", t)
        vid = t
    }
    if (match($0, /ATTRS\{idProduct\}=="[^"]*"/)) {
        t = substr($0, RSTART, RLENGTH)
        sub(/^[^=]*==/, "", t); gsub(/"/, "", t)
        pid = t
    }
    if (match($0, /ATTRS\{serial\}=="[^"]*"/)) {
        t = substr($0, RSTART, RLENGTH)
        sub(/^[^=]*==/, "", t); gsub(/"/, "", t)
        serial = t
    }
}
END { flush() }
AWK
)"
    kv="$(printf '%s\n' "$out" | awk "$AWK_PROG")"
    [ -n "$kv" ] || { err "未能从 $port 解析出设备信息"; return 1; }

    while IFS='=' read -r k v; do
        case "$k" in
            VID)    VID="$v" ;;
            PID)    PID="$v" ;;
            SERIAL) SERIAL="$v" ;;
            POS)    POS="$v" ;;
        esac
    done <<< "$kv"
    return 0
}

# ---- 根据读取到的信息生成 udev 规则文本 ----
build_rule() {
    local r='SUBSYSTEM=="tty"'
    if [ "$FIX_POSITION" = "1" ] && [ -n "$POS" ]; then
        r="$r, KERNELS==\"$POS\""
    fi
    r="$r, ATTRS{idVendor}==\"$VID\""
    r="$r, ATTRS{idProduct}==\"$PID\""
    if [ -n "$SERIAL" ]; then
        r="$r, ATTRS{serial}==\"$SERIAL\""
    fi
    r="$r, SYMLINK+=\"$NEW_NAME\""
    if [ -n "$RULE_MODE" ]; then
        r="$r, MODE=\"$RULE_MODE\""
    fi
    printf '%s' "$r"
}

# ---- 在脚本同目录查找规则文件: 存在则输出路径, 不存在返回 1 ----
find_rules_file() {
    local files=() f i choice
    if [ -f "$SCRIPT_DIR/$RULES_FILE_NAME" ]; then
        printf '%s\n' "$SCRIPT_DIR/$RULES_FILE_NAME"
        return 0
    fi
    shopt -s nullglob
    files=( "$SCRIPT_DIR"/*.rules )
    shopt -u nullglob
    if [ "${#files[@]}" -eq 0 ]; then
        return 1
    fi
    if [ "${#files[@]}" -eq 1 ]; then
        printf '%s\n' "${files[0]}"
        return 0
    fi
    # 多个规则文件 -> 交互选择
    if [ ! -t 0 ]; then
        err "同目录下存在多个规则文件,且当前无交互终端,无法选择"
        err "请在配置区设置 RULES_FILE_NAME 指定要使用的文件,或删除多余规则文件"
        return 1
    fi
    warn "同目录下检测到多个规则文件,请选择要写入的文件:"
    i=1
    for f in "${files[@]}"; do
        printf '  %d. %s\n' "$i" "$(basename "$f")"
        i=$((i + 1))
    done
    while :; do
        read -r -p "请输入序号: " choice
        case "$choice" in
            ''|*[!0-9]*) err "输入无效,请重新输入" ;;
            *)
                if [ "$choice" -ge 1 ] && [ "$choice" -le "${#files[@]}" ]; then
                    printf '%s\n' "${files[$((choice - 1))]}"
                    return 0
                fi
                err "输入无效,请重新输入"
                ;;
        esac
    done
}

# ---- 同目录规则文件: 存在则新增一条规则, 不存在则新建。输出规则文件路径 ----
append_or_create_rule() {
    local rule="$1" f
    local comment="# $NEW_NAME (由 serial_rename.sh 自动生成, $(date '+%Y-%m-%d %H:%M:%S'))"

    if f="$(find_rules_file)"; then
        # 去重: 相同规则不再重复添加
        if grep -Fq -- "$rule" "$f"; then
            warn "该规则已存在于 $(basename "$f"),跳过新增"
            printf '%s\n' "$f"
            return 0
        fi
        printf '\n%s\n%s\n' "$comment" "$rule" >> "$f"
        ok "已在现有规则文件中新增一条规则: $f"
        printf '%s\n' "$f"
    else
        f="$SCRIPT_DIR/$RULES_FILE_NAME"
        {
            echo "# 串口重命名规则(由 serial_rename.sh 自动生成)"
            echo "# 修改后执行: sudo udevadm control --reload-rules && sudo udevadm trigger"
            printf '%s\n%s\n' "$comment" "$rule"
        } > "$f"
        ok "未发现现有规则文件,已新建并添加规则: $f"
        printf '%s\n' "$f"
    fi
}

# ---- 复制规则到系统目录, 重载规则并刷新串口 ----
install_and_reload() {
    local f="$1"
    local dest="$RULES_DIR/$(basename "$f")"

    if ! cp -f "$f" "$dest" 2>/dev/null; then
        err "无法写入 $RULES_DIR,请确认以 root 运行"
        exit 1
    fi
    ok "规则已复制到系统规则目录: $dest"

    step "重新载入 udev 规则..."
    if ! udevadm control --reload-rules 2>/dev/null; then
        # 兼容旧版 udevadm 的下划线参数
        udevadm control --reload_rules 2>/dev/null \
            || warn "重载规则失败,请手动执行: sudo udevadm control --reload-rules"
    fi

    step "刷新串口设备..."
    udevadm trigger 2>/dev/null || warn "触发设备刷新失败,请手动执行: sudo udevadm trigger"
    # 针对目标设备再触发一次,确保软链接立即生成
    local syspath="/sys/class/tty/$(basename "$CURRENT_PORT")"
    [ -e "$syspath" ] && udevadm trigger --action=change "$syspath" 2>/dev/null
    udevadm settle 2>/dev/null
}

# ---- 验证新名称是否生效 ----
verify() {
    step "等待 udev 处理..."
    sleep 2
    if [ -L "/dev/$NEW_NAME" ] || [ -e "/dev/$NEW_NAME" ]; then
        ok "新串口名称已生效: /dev/$NEW_NAME -> $(readlink -f "/dev/$NEW_NAME")"
    else
        warn "尚未检测到 /dev/$NEW_NAME"
        echo "       请重新插拔设备,或手动执行:"
        echo "       sudo udevadm control --reload-rules && sudo udevadm trigger"
    fi
}

# ---- 主流程 ----
main() {
    # 命令行参数可覆盖配置: ./serial_rename.sh [串口] [新名称]
    if [ $# -ge 1 ] && [ -n "$1" ]; then CURRENT_PORT="$1"; fi
    if [ $# -ge 2 ] && [ -n "$2" ]; then NEW_NAME="$2"; fi

    echo "========================================================"
    echo "串口重命名脚本"
    echo "========================================================"

    ensure_root "$@"

    # 1. 确定目标串口
    shopt -s nullglob
    PORTS=( /dev/ttyUSB* /dev/ttyACM* )
    shopt -u nullglob

    if [ -n "$CURRENT_PORT" ] && [[ "$CURRENT_PORT" != /dev/* ]]; then
        CURRENT_PORT="/dev/$CURRENT_PORT"
    fi
    if [ -z "$CURRENT_PORT" ]; then
        [ "${#PORTS[@]}" -gt 0 ] || { err "未检测到任何串口(/dev/ttyUSB* /dev/ttyACM*),请先插入串口设备"; exit 1; }
        if [ ! -t 0 ]; then
            err "检测到多个串口且当前无交互终端,无法选择"
            err "请在配置区设置 CURRENT_PORT,或用命令行参数指定串口"
            exit 1
        fi
        log "检测到以下串口,请输入序号选择:"
        local i=1 choice
        for p in "${PORTS[@]}"; do
            printf '  %d. %s\n' "$i" "$p"
            i=$((i + 1))
        done
        while :; do
            read -r -p "请输入序号: " choice
            case "$choice" in
                ''|*[!0-9]*) err "输入无效,请重新输入" ;;
                *)
                    if [ "$choice" -ge 1 ] && [ "$choice" -le "${#PORTS[@]}" ]; then
                        break
                    fi
                    err "输入无效,请重新输入"
                    ;;
            esac
        done
        CURRENT_PORT="${PORTS[$((choice - 1))]}"
    fi

    if [ ! -e "$CURRENT_PORT" ]; then
        err "配置的串口不存在: $CURRENT_PORT"
        if [ "${#PORTS[@]}" -gt 0 ]; then
            echo "       当前检测到的串口: ${PORTS[*]}"
            echo "       请修改脚本顶部的 CURRENT_PORT 配置"
        else
            echo "       未检测到任何串口,请先插入串口设备"
        fi
        exit 1
    fi

    # 校验新名称
    case "$NEW_NAME" in
        ''|*/*|*' '*)
            err "NEW_NAME 配置无效: '$NEW_NAME' (应为不含 / 和空格的名称)"
            exit 1
            ;;
    esac

    # 2. 读取串口信息
    log "正在读取串口信息: $CURRENT_PORT"
    if ! get_port_info "$CURRENT_PORT"; then
        exit 1
    fi
    printf '  VID      : %s\n' "${VID:-未识别}"
    printf '  PID      : %s\n' "${PID:-未识别}"
    printf '  序列号   : %s\n' "${SERIAL:-无}"
    printf '  端口位置 : %s\n' "${POS:-未识别}"

    if [ -z "$VID" ] || [ -z "$PID" ]; then
        err "未能获取到设备的 VID/PID,无法生成规则"
        exit 1
    fi
    if [ -z "$SERIAL" ]; then
        warn "该设备没有序列号,规则将仅按 VID/PID 匹配"
        warn "若会插入多个同型号设备,建议开启 FIX_POSITION=1 固定位置"
    fi
    if [ "$FIX_POSITION" = "1" ] && [ -z "$POS" ]; then
        warn "已开启 FIX_POSITION,但未能获取端口位置,规则中将不包含位置信息"
    fi

    # 3. 生成规则
    local rule
    rule="$(build_rule)"
    echo ""
    log "生成的规则:"
    printf '  %s\n' "$rule"

    if [ "$ASK_CONFIRM" = "1" ]; then
        read -r -p $'\n确认将该规则写入规则文件并应用到系统? [Y/n]: ' ans
        case "$ans" in
            ''|y|Y|yes|YES) ;;
            *) echo "已取消"; exit 0 ;;
        esac
    fi

    # 4. 判断同目录规则文件: 存在则新增, 不存在则新建
    echo ""
    step "检查脚本同目录下的规则文件..."
    local rules_file
    rules_file="$(append_or_create_rule "$rule")"

    # 5. 复制规则到系统目录, 重载并刷新
    echo ""
    step "部署规则到系统..."
    install_and_reload "$rules_file"

    # 6. 刷新串口并验证
    echo ""
    step "刷新串口..."
    verify

    echo ""
    ok "串口重命名流程结束"
    echo "       新名称: /dev/$NEW_NAME (指向 $CURRENT_PORT)"
    echo "       规则文件: $SCRIPT_DIR/$(basename "$rules_file")"
}

# ==============================================================================
# 支持被其他脚本 source(仅加载函数,不执行主流程):
#   SERIAL_RENAME_SOURCE_ONLY=1 source ./serial_rename.sh
# ==============================================================================
if [ "${SERIAL_RENAME_SOURCE_ONLY:-0}" = "1" ]; then
    if [ "${BASH_SOURCE[0]}" != "$0" ]; then
        return 0
    fi
    exit 0
fi

main "$@"
