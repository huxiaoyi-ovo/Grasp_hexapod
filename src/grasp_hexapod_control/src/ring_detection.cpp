// Native equivalent of dock_mode.py's complete/partial concentric-ring detector.
#include "grasp_hexapod_control_cpp/ring_detection.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <tuple>
#include <vector>

namespace grasp_hexapod_ring {
namespace {
constexpr std::array<double, 8> diameters{.060,.080,.092,.112,.126,.146,.162,.182};
constexpr double outerDiameter = .182;
constexpr double pinDiameter = .0435;
struct Candidate { cv::Point2d center; double radius, residual, coverage; };
double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const auto n = values.size();
  return n % 2 ? values[n/2] : .5 * (values[n/2-1] + values[n/2]);
}
void fitArc(const std::vector<cv::Point2d>& points, int first, int last,
            int maxDimension, std::vector<Candidate>& candidates) {
  const int n = last-first;
  if (n < 30) return;
  cv::Mat a(n,3,CV_64F), rhs(n,1,CV_64F), abc;
  for (int i=0;i<n;++i) {
    const auto& p=points[first+i];
    a.at<double>(i,0)=p.x; a.at<double>(i,1)=p.y; a.at<double>(i,2)=1;
    rhs.at<double>(i)=-p.dot(p);
  }
  if (!cv::solve(a,rhs,abc,cv::DECOMP_SVD)) return;
  cv::Point2d center(-.5*abc.at<double>(0),-.5*abc.at<double>(1));
  const double r2=center.dot(center)-abc.at<double>(2);
  if (!(r2>0)) return;
  const double radius=std::sqrt(r2);
  std::vector<double> errors; errors.reserve(n);
  double previous=0, unwrapped=0, low=0, high=0;
  for(int i=0;i<n;++i) {
    const auto p=points[first+i]-center;
    errors.push_back(std::abs(cv::norm(p)-radius));
    const double phase=std::atan2(p.y,p.x);
    if(i==0) low=high=unwrapped=phase;
    else {
      double delta=phase-previous;
      if(delta>M_PI) delta-=2*M_PI;
      if(delta<-M_PI) delta+=2*M_PI;
      unwrapped+=delta;
      low=std::min(low,unwrapped); high=std::max(high,unwrapped);
    }
    previous=phase;
  }
  const double residual=median(std::move(errors)), coverage=high-low;
  if(radius>20 && radius<8*maxDimension && residual<=3 && coverage>=.08)
    candidates.push_back({center,radius,residual,coverage});
}
std::vector<Candidate> circleCandidates(const cv::Mat& edges) {
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(edges,contours,cv::RETR_LIST,cv::CHAIN_APPROX_NONE);
  std::vector<Candidate> candidates;
  for(const auto& contour:contours) {
    std::vector<std::vector<cv::Point2d>> arcs(1);
    bool previousInside=false;
    for(const auto& p:contour) {
      const bool inside=p.x>3 && p.x<edges.cols-4 && p.y>3 && p.y<edges.rows-4;
      if(inside) {
        if(!previousInside && !arcs.back().empty()) arcs.emplace_back();
        arcs.back().emplace_back(p.x,p.y);
      }
      previousInside=inside;
    }
    for(const auto& arc:arcs) {
      const int n=arc.size(); if(n<30) continue;
      const int step=std::max(3,std::min(10,n/30));
      std::vector<std::pair<double,int>> corners;
      for(int i=step+1;i<n-step;++i) {
        const auto incoming=arc[i]-arc[(i-step+n)%n];
        const auto outgoing=arc[(i+step)%n]-arc[i];
        double cosine=incoming.dot(outgoing)/(cv::norm(incoming)*cv::norm(outgoing)+1e-9);
        if(cosine<.45) corners.emplace_back(cosine,i);
      }
      std::stable_sort(corners.begin(),corners.end());
      std::vector<int> selected;
      for(const auto& item:corners) {
        if(std::all_of(selected.begin(),selected.end(),[&](int old){return std::abs(item.second-old)>2*step;}))
          selected.push_back(item.second);
      }
      std::sort(selected.begin(),selected.end());
      selected.insert(selected.begin(),0); selected.push_back(n);
      for(size_t i=1;i<selected.size();++i) {
        int start=selected[i-1],stop=selected[i];
        fitArc(arc,start+(start?step:0),stop-(stop<n?step:0),std::max(edges.rows,edges.cols),candidates);
      }
    }
  }
  std::stable_sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){
    return std::make_pair(-a.coverage,a.residual)<std::make_pair(-b.coverage,b.residual);
  });
  std::vector<Candidate> unique;
  for(const auto& item:candidates) {
    if(std::any_of(unique.begin(),unique.end(),[&](const Candidate& old){
      return cv::norm(item.center-old.center)<4 && std::abs(item.radius-old.radius)<.015*old.radius;
    })) continue;
    unique.push_back(item); if(unique.size()==32) break;
  }
  return unique;
}
struct Support { int strong=0,colors=0; double sum=0; };
Support support(const cv::Mat& distance,const cv::Mat& gray,double threshold,cv::Point2d center,double radius) {
  static const auto unit=[] {
    std::array<cv::Point2d,360> points{};
    for(int i=0;i<360;++i) { const double t=i*2*M_PI/360.;points[i]={std::cos(t),std::sin(t)}; }
    return points;
  }();
  Support result;
  for(int ring=0;ring<8;++ring) {
    double ratio=diameters[ring]/outerDiameter;
    double inner=(ring?diameters[ring-1]:pinDiameter)/outerDiameter;
    int visible=0,hits=0,colorVisible=0,colors=0;
    for(const auto& p:unit) {
      int x=std::nearbyint(center.x+radius*ratio*p.x), y=std::nearbyint(center.y+radius*ratio*p.y);
      if(x>0 && x<gray.cols-1 && y>0 && y<gray.rows-1) {
        ++visible; hits+=distance.at<float>(y,x)<=3;
      }
      x=std::nearbyint(center.x+radius*.5*(inner+ratio)*p.x);
      y=std::nearbyint(center.y+radius*.5*(inner+ratio)*p.y);
      if(x>0 && x<gray.cols-1 && y>0 && y<gray.rows-1) {
        ++colorVisible; colors+=((gray.at<unsigned char>(y,x)>threshold)==(ring%2==0));
      }
    }
    if(visible>=8) { double fraction=double(hits)/visible;result.sum+=fraction;result.strong+=fraction>=.28; }
    if(colorVisible>=8) result.colors+=double(colors)/colorVisible>=.65;
  }
  return result;
}
}
RingDetection detectRings(const cv::Mat& gray) {
  RingDetection result;
  if(gray.type()!=CV_8UC1) throw std::invalid_argument("ring image must be mono8");
  if(std::min(gray.rows,gray.cols)<64) {result.reason="image is too small";return result;}
  cv::Mat blurred,binary,edges,distance;
  cv::GaussianBlur(gray,blurred,{5,5},0);
  double threshold=cv::threshold(blurred,binary,0,255,cv::THRESH_BINARY|cv::THRESH_OTSU);
  cv::Canny(blurred,edges,std::max(20,int(.35*threshold)),std::max(60,int(.9*threshold)));
  const auto candidates=circleCandidates(edges);
  if(candidates.empty()) {result.reason="no usable ring arcs";return result;}
  cv::distanceTransform(255-edges,distance,cv::DIST_L2,3);
  std::tuple<int,int,double,double> best;
  for(const auto& c:candidates) for(double diameter:diameters) {
    double radius=c.radius/(diameter/outerDiameter);
    if(2*radius<.35*std::min(gray.rows,gray.cols)) continue;
    auto s=support(distance,blurred,threshold,c.center,radius);
    auto score=std::make_tuple(s.strong,s.colors,s.sum,-c.residual);
    if(s.strong>=3 && s.colors>=3 && (!result.valid || score>best)) {
      best=score; result.valid=true;result.center=c.center;result.diameter=2*radius;
      result.matched=s.strong;result.ratio_error=std::max(0.,1-s.sum/s.strong);
    }
  }
  result.reason=result.valid?std::to_string(result.matched)+" ring boundaries":"fewer than three credible ring boundaries";
  return result;
}
}
