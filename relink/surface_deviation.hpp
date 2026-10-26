#pragma once
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <algorithm>
#include <array>
#include <vector>
#include <limits>
#include <cmath>
// Reusable nearest-triangle BVH. Magnitude is Euclidean distance, sign is
// determined by ray parity against a closed reference, independent of normals.
class CNCSurfaceDeviation : public godot::RefCounted {
 GDCLASS(CNCSurfaceDeviation,godot::RefCounted)
 using V=godot::Vector3;
 struct Tri {V a,b,c,low,high,center;};
 struct Node {V low,high;int left=-1,right=-1,start=0,count=0;};
 std::vector<Tri> triangles;
 std::vector<Node> nodes;
 double epsilon=1e-8;
 static V low(V a,V b){return V(std::min(a.x,b.x),std::min(a.y,b.y),std::min(a.z,b.z));}
 static V high(V a,V b){return V(std::max(a.x,b.x),std::max(a.y,b.y),std::max(a.z,b.z));}
 int build(int begin,int end){
  Node node;node.low=V(INFINITY,INFINITY,INFINITY);node.high=-node.low;
  for(int i=begin;i<end;++i){node.low=low(node.low,triangles[i].low);node.high=high(node.high,triangles[i].high);}
  int index=nodes.size();nodes.push_back(node);
  if(end-begin<=8){nodes[index].start=begin;nodes[index].count=end-begin;return index;}
  V span=node.high-node.low;int axis=span.x>span.y?(span.x>span.z?0:2):(span.y>span.z?1:2);int mid=(begin+end)/2;
  std::nth_element(triangles.begin()+begin,triangles.begin()+mid,triangles.begin()+end,[axis](const Tri&a,const Tri&b){return a.center[axis]<b.center[axis];});
  int l=build(begin,mid),r=build(mid,end);nodes[index].left=l;nodes[index].right=r;return index;
 }
 static V closest(V p,const Tri&t){
  V ab=t.b-t.a,ac=t.c-t.a,ap=p-t.a;
  double d1=ab.dot(ap),d2=ac.dot(ap);if(d1<=0&&d2<=0)return t.a;
  V bp=p-t.b;double d3=ab.dot(bp),d4=ac.dot(bp);if(d3>=0&&d4<=d3)return t.b;
  double vc=d1*d4-d3*d2;if(vc<=0&&d1>=0&&d3<=0)return t.a+ab*(d1/(d1-d3));
  V cp=p-t.c;double d5=ab.dot(cp),d6=ac.dot(cp);if(d6>=0&&d5<=d6)return t.c;
  double vb=d5*d2-d1*d6;if(vb<=0&&d2>=0&&d6<=0)return t.a+ac*(d2/(d2-d6));
  double va=d3*d6-d5*d4;if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0)return t.b+(t.c-t.b)*((d4-d3)/((d4-d3)+(d5-d6)));
  double inv=1./(va+vb+vc);return t.a+ab*(vb*inv)+ac*(vc*inv);
 }
 static double box_distance(V p,const Node&n){
  double result=0;for(int a=0;a<3;++a){double d=std::max({double(n.low[a]-p[a]),double(p[a]-n.high[a]),0.});result+=d*d;}return result;
 }
 void nearest(int index,V p,double&best,V&point)const{
  const auto &node=nodes[index];if(box_distance(p,node)>best)return;
  if(node.count){for(int i=node.start;i<node.start+node.count;++i){V q=closest(p,triangles[i]);double d=p.distance_squared_to(q);if(d<best){best=d;point=q;}}return;}
  int l=node.left,r=node.right;if(box_distance(p,nodes[l])>box_distance(p,nodes[r]))std::swap(l,r);
  nearest(l,p,best,point);nearest(r,p,best,point);
 }
 static bool ray_box(V p,V direction,const Node&n){
  double lo=0,hi=INFINITY;
  for(int a=0;a<3;++a){double u=(n.low[a]-p[a])/direction[a],v=(n.high[a]-p[a])/direction[a];if(u>v)std::swap(u,v);lo=std::max(lo,u);hi=std::min(hi,v);if(hi<lo)return false;}
  return true;
 }
 void ray_hits(int index,V p,V direction,std::vector<double>&hits)const{
  const auto&node=nodes[index];if(!ray_box(p,direction,node))return;
  if(!node.count){ray_hits(node.left,p,direction,hits);ray_hits(node.right,p,direction,hits);return;}
  for(int i=node.start;i<node.start+node.count;++i){
   const auto&t=triangles[i];V ab=t.b-t.a,ac=t.c-t.a,h=direction.cross(ac);double det=ab.dot(h);if(std::abs(det)<1e-13)continue;
   V s=p-t.a;double inv=1./det,u=s.dot(h)*inv;if(u<0||u>1)continue;
   V q=s.cross(ab);double v=direction.dot(q)*inv;if(v<0||u+v>1)continue;
   double distance=ac.dot(q)*inv;if(distance>epsilon)hits.push_back(distance);
  }
 }
 double distance(V p,V&point)const{
  double best=INFINITY;nearest(0,p,best,point);double magnitude=std::sqrt(best);if(magnitude<epsilon)return 0.;
  std::vector<double>hits;ray_hits(0,p,V(1.,.371390676,.529173125).normalized(),hits);std::sort(hits.begin(),hits.end());
  int count=0;double previous=-INFINITY;
  for(double h:hits){if(h-previous>epsilon*4)++count;previous=h;}
  return count%2?-magnitude:magnitude;
 }
protected:
 static void _bind_methods(){
  godot::ClassDB::bind_method(godot::D_METHOD("set_reference","vertices","indices"),&CNCSurfaceDeviation::set_reference);
  godot::ClassDB::bind_method(godot::D_METHOD("sample","points"),&CNCSurfaceDeviation::sample);
 }
public:
 bool set_reference(const godot::PackedVector3Array&vertices,const godot::PackedInt32Array&indices){
  triangles.clear();nodes.clear();if(indices.size()%3)return false;
  for(int i=0;i<indices.size();i+=3){
   for(int j=0;j<3;++j)if(indices[i+j]<0||indices[i+j]>=vertices.size())return false;
   V a=vertices[indices[i]],b=vertices[indices[i+1]],c=vertices[indices[i+2]];
   if(!a.is_finite()||!b.is_finite()||!c.is_finite())return false;
   if((b-a).cross(c-a).length_squared()<1e-22)continue;
   triangles.push_back({a,b,c,low(a,low(b,c)),high(a,high(b,c)),(a+b+c)/3.});
  }
  if(triangles.empty())return false;
  build(0,triangles.size());epsilon=std::max(1e-9,double((nodes[0].high-nodes[0].low).length())*1e-8);return true;
 }
 godot::Dictionary sample(const godot::PackedVector3Array&points)const{
  godot::Dictionary result;result["valid"]=!nodes.empty();if(nodes.empty())return result;
  godot::PackedFloat32Array values;godot::PackedVector3Array closest_points;
  values.resize(points.size());closest_points.resize(points.size());
  for(int i=0;i<points.size();++i){V q;values.set(i,distance(points[i],q));closest_points.set(i,q);}
  result["distances"]=values;result["closest_points"]=closest_points;return result;
 }
};
