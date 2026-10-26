#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <manifold/manifold.h>
#include <clipper2/clipper.h>
#include "batchdropcutter.hpp"
#include "adaptivewaterline.hpp"
#include "ballcutter.hpp"
#include "cylcutter.hpp"
#include "stlsurf.hpp"
// Web build uses Manifold serial backend, no TBB runtime.
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include "surface_deviation.hpp"
using namespace godot;
class CNCManifold : public RefCounted {
 GDCLASS(CNCManifold, RefCounted)
 // Parallelism disabled in this Web proof.
 manifold::Manifold solid;
 manifold::Manifold target;
 manifold::Manifold last_sweep;
 manifold::Manifold viewer_baseline;
 PackedVector3Array viewer_points;
 bool viewer_valid=false;
 bool stream_sweep_cache=false;
 bool incremental_stream_cut=false;
 bool air_grid_enabled=true;
 double air_projection_pad=0;
 double air_cell=0;manifold::vec3 air_origin;
 int air_nx=0,air_ny=0,air_nz=0,air_grid_builds=0,air_skipped=0;
 std::vector<uint8_t> air_cells;
 struct AirBox {manifold::vec3 low,high;int triangle=-1;};
 struct AirTriangle {manifold::vec3 p[3];};
 struct AirPlane {manifold::vec3 normal;double offset;};
 std::vector<AirTriangle> air_triangles;
 int air_convex_skipped=0;
 struct AirNode {AirBox box;int start=0,count=0,left=-1,right=-1;};
 std::vector<AirBox> air_surfaces;std::vector<AirNode> air_nodes;
 manifold::vec3 air_model_low,air_model_high;
 struct SweepPrefix {PackedVector3Array points;manifold::Manifold solid;bool has=false;};
 std::vector<SweepPrefix> sweep_prefixes;
 int prefix_hits=0;
 double cached_radius=-1,cached_length=-1,cached_tolerance=-1;bool cached_ball=false;
 std::vector<manifold::Manifold> configured_parts;
 std::vector<std::vector<manifold::vec3>> configured_vertices;
 double last_ms = 0;
 double hull_ms=0,boolean_ms=0,export_ms=0;
 int sweep_count=0;
 double compression_error=0;
 int input_points=0,kept_points=0;
 // Opt-in contact query cache, owned by one CNCManifold query instance.
 // Declaration order keeps the kd-tree alive only while its surface exists.
 PackedVector3Array contact_vertices;
 PackedInt32Array contact_indices;
 std::unique_ptr<ocl::STLSurf> contact_surface;
 std::unique_ptr<ocl::BatchDropCutter> contact_operation;
 int contact_builds=0,contact_hits=0;
protected:
 static void _bind_methods() {
  ClassDB::bind_method(D_METHOD("set_parallel_threads", "count"), &CNCManifold::set_parallel_threads);
  ClassDB::bind_method(D_METHOD("set_viewer_compression", "error"), &CNCManifold::set_viewer_compression);
  ClassDB::bind_method(D_METHOD("set_stream_sweep_cache", "enabled"), &CNCManifold::set_stream_sweep_cache);
  ClassDB::bind_method(D_METHOD("set_incremental_stream_cut", "enabled"), &CNCManifold::set_incremental_stream_cut);
  ClassDB::bind_method(D_METHOD("set_air_grid_culling", "enabled"), &CNCManifold::set_air_grid_culling);
  ClassDB::bind_method(D_METHOD("finish_stream_segment", "radius", "length", "ball", "tolerance"), &CNCManifold::finish_stream_segment);
  ClassDB::bind_method(D_METHOD("performance_stats"), &CNCManifold::performance_stats);
  ClassDB::bind_method(D_METHOD("surface_path_cached", "vertices", "indices", "samples", "radius"), &CNCManifold::surface_path_cached);
  ClassDB::bind_method(D_METHOD("surface_path_flat_cached", "vertices", "indices", "samples", "radius"), &CNCManifold::surface_path_flat_cached);
  ClassDB::bind_method(D_METHOD("contact_cache_stats"), &CNCManifold::contact_cache_stats);
  ClassDB::bind_method(D_METHOD("surface_path", "vertices", "indices", "samples", "radius"), &CNCManifold::surface_path);
  ClassDB::bind_method(D_METHOD("waterline_paths", "vertices", "indices", "heights", "radius", "sampling"), &CNCManifold::waterline_paths);
  ClassDB::bind_method(D_METHOD("waterline_paths_reused", "vertices", "indices", "heights", "radius", "sampling"), &CNCManifold::waterline_paths_reused);
  ClassDB::bind_method(D_METHOD("waterline_paths_parallel", "vertices", "indices", "heights", "radius", "sampling", "workers"), &CNCManifold::waterline_paths_parallel);
  ClassDB::bind_method(D_METHOD("pocket_paths", "axis", "heights", "radius", "stepover", "allowance", "layer_depth"), &CNCManifold::pocket_paths);
  ClassDB::bind_method(D_METHOD("pocket_detail_paths", "axis", "heights", "radius", "stepover", "allowance", "layer_depth"), &CNCManifold::pocket_detail_paths);
  ClassDB::bind_method(D_METHOD("surface_path_flat", "vertices", "indices", "samples", "radius"), &CNCManifold::surface_path_flat);
  ClassDB::bind_method(D_METHOD("cut_ball", "points", "radius"), &CNCManifold::cut_ball);
  ClassDB::bind_method(D_METHOD("load_stock", "vertices", "indices"), &CNCManifold::load_stock);
  ClassDB::bind_method(D_METHOD("copy_state_from", "source"), &CNCManifold::copy_state_from);
  ClassDB::bind_method(D_METHOD("apply_mesh_boolean", "vertices", "indices", "subtract"), &CNCManifold::apply_mesh_boolean);
  ClassDB::bind_method(D_METHOD("configure_viewer_tool", "axis", "config", "tolerance"), &CNCManifold::configure_viewer_tool);
  ClassDB::bind_method(D_METHOD("get_configured_tool"), &CNCManifold::get_configured_tool);
  ClassDB::bind_method(D_METHOD("begin_viewer_segment"), &CNCManifold::begin_viewer_segment);
  ClassDB::bind_method(D_METHOD("cut_viewer_rows", "rows", "radius", "length", "ball", "tolerance"), &CNCManifold::cut_viewer_rows);
  ClassDB::bind_method(D_METHOD("get_last_sweep"), &CNCManifold::get_last_sweep);
  ClassDB::bind_method(D_METHOD("reset_stock", "size", "center"), &CNCManifold::reset_stock);
  ClassDB::bind_method(D_METHOD("cut_viewer", "points", "radius", "length", "ball", "tolerance"), &CNCManifold::cut_viewer);
  ClassDB::bind_method(D_METHOD("reset"), &CNCManifold::reset);
  ClassDB::bind_method(D_METHOD("reset_plate"), &CNCManifold::reset_plate);
  ClassDB::bind_method(D_METHOD("set_target", "vertices", "indices"), &CNCManifold::set_target);
  ClassDB::bind_method(D_METHOD("inspect_target"), &CNCManifold::inspect_target);
  ClassDB::bind_method(D_METHOD("inspect_target_reference"), &CNCManifold::inspect_target_reference);
  ClassDB::bind_method(D_METHOD("inspect_target_reference_missing"), &CNCManifold::inspect_target_reference_missing);
  ClassDB::bind_method(D_METHOD("get_reference_difference_mesh", "missing"), &CNCManifold::get_reference_difference_mesh);
  ClassDB::bind_method(D_METHOD("get_difference_mesh", "missing"), &CNCManifold::get_difference_mesh);
  ClassDB::bind_method(D_METHOD("excess_in_mesh", "vertices", "indices"), &CNCManifold::excess_in_mesh);
  ClassDB::bind_method(D_METHOD("material_in_box", "minimum", "maximum"), &CNCManifold::material_in_box);
  ClassDB::bind_method(D_METHOD("cut_batch", "points", "radius"), &CNCManifold::cut_batch);
  ClassDB::bind_method(D_METHOD("get_display_mesh"), &CNCManifold::get_display_mesh);
  ClassDB::bind_method(D_METHOD("get_mesh"), &CNCManifold::get_mesh);
  ClassDB::bind_method(D_METHOD("get_mesh_arrays"), &CNCManifold::get_mesh_arrays);
  ClassDB::bind_method(D_METHOD("get_compact_mesh_arrays"), &CNCManifold::get_compact_mesh_arrays);
  ClassDB::bind_method(D_METHOD("volume"), &CNCManifold::volume);
  ClassDB::bind_method(D_METHOD("last_milliseconds"), &CNCManifold::last_milliseconds);
  ClassDB::bind_method(D_METHOD("status"), &CNCManifold::status);
  ClassDB::bind_method(D_METHOD("inspect"), &CNCManifold::inspect);
  ClassDB::bind_method(D_METHOD("get_stock_components"), &CNCManifold::get_stock_components);
  ClassDB::bind_method(D_METHOD("retain_at", "point"), &CNCManifold::retain_at);
 }
public:
 CNCManifold() { reset(); }
 void set_parallel_threads(int count){(void)count;}
 void set_air_grid_culling(bool enabled){air_grid_enabled=enabled;if(!enabled)air_cells.clear();}
 void set_incremental_stream_cut(bool enabled){if(enabled!=incremental_stream_cut)sweep_prefixes.clear();incremental_stream_cut=enabled;}
 void set_stream_sweep_cache(bool enabled){if(enabled!=stream_sweep_cache)sweep_prefixes.clear();stream_sweep_cache=enabled;}
 void set_viewer_compression(double error){compression_error=std::max(0.0,error);}
 Dictionary performance_stats()const{Dictionary d;d["hull_ms"]=hull_ms;d["boolean_ms"]=boolean_ms;d["export_ms"]=export_ms;d["sweeps"]=sweep_count;d["prefix_hits"]=prefix_hits;d["air_grid_builds"]=air_grid_builds;d["air_skipped_sweeps"]=air_skipped;d["air_convex_skipped"]=air_convex_skipped;d["input_points"]=input_points;d["kept_points"]=kept_points;return d;}
 Dictionary pocket_paths(Vector3 axis,const Array &heights,double radius,double stepover,double allowance,double layer_depth) const {
  return pocket_paths_impl(axis,heights,radius,stepover,allowance,layer_depth,false);
 }
 Dictionary pocket_detail_paths(Vector3 axis,const Array &heights,double radius,double stepover,double allowance,double layer_depth) const {
  return pocket_paths_impl(axis,heights,radius,stepover,allowance,layer_depth,true);
 }
 Dictionary pocket_paths_impl(Vector3 axis,const Array &heights,double radius,double stepover,double allowance,double layer_depth,bool enclosed_only) const {
  namespace C2=Clipper2Lib;
  Dictionary out;Array rows;double estimate=0;
  if(axis.length()<.99 || radius<=0 || stepover<=0 || heights.size()>256){out["error"]="Invalid pocket parameters";return out;}
  axis=axis.normalized();Vector3 x=std::abs(axis.dot(Vector3(1,0,0)))<.9 ? Vector3(1,0,0):Vector3(0,1,0);Vector3 z=x.cross(axis);
  // Proper rotation: world -> local (X, -Z, Y), with machining axis on Z.
  manifold::mat3x4 frame{{x.x,-z.x,axis.x},{x.y,-z.y,axis.y},{x.z,-z.z,axis.z},{0,0,0}};
  auto stock=reference_body(solid).Transform(frame),goal=reference_body(target).Transform(frame);
  auto remaining=stock-goal;
  auto polygons=[](const manifold::Polygons &p){C2::PathsD result;for(auto &loop:p){C2::PathD path;for(auto v:loop)path.emplace_back(v.x,v.y);result.push_back(path);}return C2::Union(result,C2::FillRule::Positive,6);};
  auto stock_outline=polygons(stock.Project());
  auto boundary=C2::InflatePaths(stock_outline,radius,C2::JoinType::Round,C2::EndType::Polygon,2.,6,.0005);
  double safe=stock.BoundingBox().max.z+.3;
  for(int hi=0;hi<heights.size();++hi){
   double h=heights[hi];
   auto obstacles=polygons(goal.TrimByPlane({0,0,1},h-allowance*.25).Project());
   auto forbidden=C2::InflatePaths(obstacles,radius+allowance,C2::JoinType::Round,C2::EndType::Polygon,2.,6,.0005);
   auto allowed=C2::Difference(boundary,forbidden,C2::FillRule::NonZero,6);
   if(enclosed_only){
    // Identify cavities in the target obstacle, not in the remaining stock:
    // after roughing, outer clearance is often split into disconnected strips.
    C2::ClipperD clipper(6);clipper.AddSubject(forbidden);
    C2::PolyTreeD tree;clipper.Execute(C2::ClipType::Union,C2::FillRule::NonZero,tree);
    C2::PathsD voids;
    std::function<void(const C2::PolyPathD&,int)> collect=[&](const C2::PolyPathD &node,int depth){
     for(size_t i=0;i<node.Count();++i){const auto &child=*node.Child(i);if(depth>=1){auto loop=child.Polygon();std::reverse(loop.begin(),loop.end());voids.push_back(std::move(loop));}collect(child,depth+1);}
    };
    collect(tree,0);
    allowed=C2::Intersect(allowed,voids,C2::FillRule::NonZero,6);
   }
   auto slab=remaining.TrimByPlane({0,0,1},h-1e-5).TrimByPlane({0,0,-1},-(h+layer_depth));
   auto material=polygons(slab.Project());
   if(material.empty())continue;
   auto reach=C2::InflatePaths(material,radius*.95,C2::JoinType::Round,C2::EndType::Polygon,2.,6,.0005);
   auto region=C2::Intersect(allowed,reach,C2::FillRule::NonZero,6);
   estimate+=std::abs(C2::Area(region))*layer_depth;
   for(int pass=0;pass<256 && !region.empty();++pass){
    auto simple=C2::SimplifyPaths(region,.0003);
    for(auto &loop:simple){
     if(loop.size()<3)continue;
     PackedVector3Array row;
     auto point=[&](C2::PointD p,double height){return x*p.x-z*p.y+axis*height;};
     row.push_back(point(loop.front(),safe));
     for(auto p:loop)row.push_back(point(p,h));
     row.push_back(point(loop.front(),h));row.push_back(point(loop.front(),safe));rows.push_back(row);
     if(rows.size()>12000){out["error"]="Pocket path budget exceeded";return out;}
    }
    region=C2::InflatePaths(simple,-stepover,C2::JoinType::Round,C2::EndType::Polygon,2.,6,.0005);
   }
  }
  out["rows"]=rows;out["estimated_volume"]=estimate;out["backend"]="Clipper2";out["enclosed_only"]=enclosed_only;return out;
 }
 Dictionary contact_cache_stats()const {
  Dictionary d;d["builds"]=contact_builds;d["hits"]=contact_hits;return d;
 }
 PackedVector3Array surface_path_cached(const PackedVector3Array &v,const PackedInt32Array &idx,const PackedVector3Array &samples,double radius) {
  return contact_path_cached(v,idx,samples,radius,false);
 }
 PackedVector3Array surface_path_flat_cached(const PackedVector3Array &v,const PackedInt32Array &idx,const PackedVector3Array &samples,double radius) {
  return contact_path_cached(v,idx,samples,radius,true);
 }
 PackedVector3Array contact_path_cached(const PackedVector3Array &v,const PackedInt32Array &idx,const PackedVector3Array &samples,double radius,bool flat) {
  if(v.is_empty() || idx.is_empty() || idx.size()%3 || radius<=0)return {};
  if(!contact_operation || contact_vertices!=v || contact_indices!=idx){
   // Recreate rather than rebuilding an existing OCL tree in place.
   contact_operation.reset();contact_surface.reset();
   contact_surface=std::make_unique<ocl::STLSurf>();
   auto convert=[](Vector3 p){return ocl::Point(p.x,p.z,p.y);};
   for(int i=0;i<idx.size();i+=3){
    for(int j=0;j<3;++j)if(idx[i+j]<0 || idx[i+j]>=v.size()){contact_surface.reset();return {};}
    contact_surface->addTriangle(ocl::Triangle(convert(v[idx[i]]),convert(v[idx[i+2]]),convert(v[idx[i+1]])));
   }
   contact_operation=std::make_unique<ocl::BatchDropCutter>();
   contact_operation->setSTL(*contact_surface);
   contact_vertices=v;contact_indices=idx;++contact_builds;
  }else ++contact_hits;
  ocl::BallCutter ball(2*radius,3.0);ocl::CylCutter cylinder(2*radius,3.0);
  contact_operation->clearCLPoints();
  contact_operation->setCutter(flat ? static_cast<ocl::MillingCutter*>(&cylinder) : static_cast<ocl::MillingCutter*>(&ball));
  for(int i=0;i<samples.size();++i){ocl::CLPoint p(samples[i].x,samples[i].z,-1.0);contact_operation->appendPoint(p);}
  contact_operation->run();PackedVector3Array out;
  for(auto &p:contact_operation->getCLPoints())out.push_back(Vector3(p.x,p.z+(flat?0.001:0.003),p.y));
  contact_operation->clearCLPoints();contact_operation->setCutter(nullptr);return out;
 }
 PackedVector3Array surface_path(const PackedVector3Array &v,const PackedInt32Array &idx,const PackedVector3Array &samples,double radius) {
  ocl::STLSurf surface;
  auto convert=[](Vector3 p){return ocl::Point(p.x,p.z,p.y);};
  for(int i=0;i+2<idx.size();i+=3) surface.addTriangle(ocl::Triangle(convert(v[idx[i]]),convert(v[idx[i+2]]),convert(v[idx[i+1]])));
  ocl::BallCutter cutter(2*radius,3.0);
  ocl::BatchDropCutter operation;
  operation.setSTL(surface); operation.setCutter(&cutter);
  for(int i=0;i<samples.size();++i) {ocl::CLPoint p(samples[i].x,samples[i].z,-1.0);operation.appendPoint(p);}
  operation.run(); PackedVector3Array out;
  for(auto &p:operation.getCLPoints()) out.push_back(Vector3(p.x,p.z+0.003,p.y));
  return out;
 }
 Array waterline_paths(const PackedVector3Array &v,const PackedInt32Array &idx,const Array &heights,double radius,double sampling) {
  return waterline_paths_impl(v,idx,heights,radius,sampling,false);
 }
 Array waterline_paths_reused(const PackedVector3Array &v,const PackedInt32Array &idx,const Array &heights,double radius,double sampling) {
  return waterline_paths_impl(v,idx,heights,radius,sampling,true);
 }
 Array waterline_paths_impl(const PackedVector3Array &v,const PackedInt32Array &idx,const Array &heights,double radius,double sampling,bool reuse) {
  Array paths;
  if(v.is_empty() || idx.size()%3 || radius<=0 || heights.size()>256)return paths;
  sampling=std::clamp(sampling,0.005,0.5);
  ocl::STLSurf surface;
  auto convert=[](Vector3 p){return ocl::Point(p.x,p.z,p.y);};
  for(int i=0;i+2<idx.size();i+=3)surface.addTriangle(ocl::Triangle(convert(v[idx[i]]),convert(v[idx[i+2]]),convert(v[idx[i+1]])));
  ocl::BallCutter cutter(2*radius,4.0);
  ocl::AdaptiveWaterline cached;
  if(reuse){cached.setSTL(surface);cached.setCutter(&cutter);cached.setSampling(sampling);cached.setMinSampling(std::max(0.001,sampling/10));}
  for(int h=0;h<heights.size();++h){
   ocl::AdaptiveWaterline fresh;
   auto &operation=reuse?cached:fresh;
   if(reuse)operation.reset();
   else{operation.setSTL(surface);operation.setCutter(&cutter);operation.setSampling(sampling);operation.setMinSampling(std::max(0.001,sampling/10));}
   operation.setZ(double(heights[h]));operation.run();
   for(auto &loop:operation.getLoops()){
    PackedVector3Array path;
    for(auto &p:loop)path.push_back(Vector3(p.x,p.z,p.y));
    if(path.size()>2){if(path[0].distance_to(path[path.size()-1])>1e-7)path.push_back(path[0]);paths.push_back(path);}
   }
  }
  return paths;
 }
 Array waterline_paths_parallel(const PackedVector3Array &v,const PackedInt32Array &idx,const Array &heights,double radius,double sampling,int workers) {
  Array paths;
  if(v.is_empty() || idx.is_empty() || idx.size()%3 || radius<=0 || heights.is_empty() || heights.size()>256)return paths;
  for(int i=0;i<idx.size();++i)if(idx[i]<0 || idx[i]>=v.size())return paths;
  sampling=std::clamp(sampling,0.005,0.5);
  ocl::STLSurf surface;
  auto convert=[](Vector3 p){return ocl::Point(p.x,p.z,p.y);};
  for(int i=0;i<idx.size();i+=3)surface.addTriangle(ocl::Triangle(convert(v[idx[i]]),convert(v[idx[i+2]]),convert(v[idx[i+1]])));
  std::vector<double> levels;
  for(int i=0;i<heights.size();++i)levels.push_back(double(heights[i]));
  using Loops=std::vector<std::vector<ocl::Point>>;
  std::vector<Loops> results(levels.size());
  int count=1; (void)workers;
  // Only native data enters workers. Each task owns its cutter, fibers and KD trees.
  // Results occupy distinct height slots and Godot containers are built after joining.
  // Serial task invocation preserves per-level OCL results.
  for(int task=0;task<count;++task){
    ocl::BallCutter cutter(2*radius,4.0);
    ocl::AdaptiveWaterline operation;
    operation.setSTL(surface);operation.setCutter(&cutter);
    operation.setSampling(sampling);operation.setMinSampling(std::max(0.001,sampling/10));
    for(size_t h=task;h<levels.size();h+=count){
     operation.reset();operation.setZ(levels[h]);operation.run();
     for(const auto &loop:operation.getLoops())results[h].emplace_back(loop.begin(),loop.end());
    }
  }
  for(const auto &level:results)for(const auto &loop:level){
   PackedVector3Array path;
   for(const auto &p:loop)path.push_back(Vector3(p.x,p.z,p.y));
   if(path.size()>2){if(path[0].distance_to(path[path.size()-1])>1e-7)path.push_back(path[0]);paths.push_back(path);}
  }
  return paths;
 }
 PackedVector3Array surface_path_flat(const PackedVector3Array &v,const PackedInt32Array &idx,const PackedVector3Array &samples,double radius) {
  ocl::STLSurf surface;
  auto convert=[](Vector3 p){return ocl::Point(p.x,p.z,p.y);};
  for(int i=0;i+2<idx.size();i+=3) surface.addTriangle(ocl::Triangle(convert(v[idx[i]]),convert(v[idx[i+2]]),convert(v[idx[i+1]])));
  ocl::CylCutter cutter(2*radius,3.0);
  ocl::BatchDropCutter operation;
  operation.setSTL(surface); operation.setCutter(&cutter);
  for(int i=0;i<samples.size();++i) {ocl::CLPoint p(samples[i].x,samples[i].z,-1.0);operation.appendPoint(p);}
  operation.run(); PackedVector3Array out;
  for(auto &p:operation.getCLPoints()) out.push_back(Vector3(p.x,p.z+0.001,p.y));
  return out;
 }
 void cut_ball(const PackedVector3Array &points,double radius) {
  auto begin=std::chrono::steady_clock::now();
  // Convex, fixed-axis ball-end tool swept along each feed segment.
  auto base=manifold::Manifold::Sphere(radius,32).Translate({0,radius,0}) + manifold::Manifold::Cylinder(3.0,radius,radius,32).Rotate(-90,0,0).Translate({0,radius,0});
  std::vector<manifold::Manifold> swept;
  for(int i=0;i<points.size();++i) {
   auto p=points[i]; auto tool=base.Translate({p.x,p.y,p.z});
   if(i>0) {auto q=points[i-1];tool=manifold::Manifold::Hull(std::vector<manifold::Manifold>{tool,base.Translate({q.x,q.y,q.z})});}
   swept.push_back(tool);
  }
  if(!swept.empty()) {auto next=solid-manifold::Manifold::BatchBoolean(swept,manifold::OpType::Add);if(next.Status()==manifold::Manifold::Error::NoError)solid=next;}
  last_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
 }
 bool copy_state_from(const Ref<CNCManifold> &source){
  if(source.is_null())return false;
  air_cells.clear();solid=source->solid;target=source->target;viewer_valid=false;
  configured_parts.clear();configured_vertices.clear();viewer_points.clear();
  last_sweep=manifold::Manifold();return true;
 }
 int load_stock(const PackedVector3Array &v,const PackedInt32Array &idx){
  manifold::MeshGL mesh;
  for(int i=0;i<v.size();++i){mesh.vertProperties.push_back(v[i].x);mesh.vertProperties.push_back(v[i].y);mesh.vertProperties.push_back(v[i].z);}
  for(int i=0;i<idx.size();++i)mesh.triVerts.push_back(idx[i]);
  mesh.Merge();auto candidate=manifold::Manifold(mesh);
  if(candidate.Status()!=manifold::Manifold::Error::NoError)return static_cast<int>(candidate.Status());
  air_cells.clear();solid=candidate;viewer_valid=false;return 0;
 }
 int apply_mesh_boolean(const PackedVector3Array &v,const PackedInt32Array &idx,bool subtract){
  manifold::MeshGL mesh;
  for(int i=0;i<v.size();++i){mesh.vertProperties.push_back(v[i].x);mesh.vertProperties.push_back(v[i].y);mesh.vertProperties.push_back(v[i].z);}
  for(int i=0;i<idx.size();++i)mesh.triVerts.push_back(idx[i]);
  mesh.Merge();auto operand=manifold::Manifold(mesh);
  if(operand.Status()!=manifold::Manifold::Error::NoError)return static_cast<int>(operand.Status());
  auto candidate=subtract ? solid-operand : solid+operand;
  if(candidate.Status()!=manifold::Manifold::Error::NoError)return static_cast<int>(candidate.Status());
  if(candidate.IsEmpty() || candidate.Volume()<1e-8)return -1;
  if(candidate.Decompose().size()!=1)return -2;
  air_cells.clear();solid=candidate;viewer_valid=false;configured_parts.clear();viewer_points.clear();last_sweep=manifold::Manifold();
  return 0;
 }
 void configure_viewer_tool(Vector3 axis,Dictionary config,double tolerance){
  air_cells.clear();
  // Rebase fine passes to the same float geometry used for display and safety
  // inspection. Long double-precision boolean histories can retain nearly
  // coincident slivers; oversized simplification tolerances also need resetting.
  if(solid.GetTolerance()>tolerance || tolerance<0.001){
   solid=reference_body(solid);viewer_valid=false;
  }
  sweep_prefixes.clear();configured_parts.clear();configured_vertices.clear();if(config.is_empty())return;
  Dictionary d=config["dimension"];String type=config["type"];
  double r=double(d["diameter"])/2,L=double(d["length"]);
  int n=std::min(256,std::max(24,int(std::ceil(3.141592653589793/std::acos(std::max(-1.0,1.0-tolerance/r))))));n=(n+3)/4*4;
  axis=axis.normalized();Vector3 x=Vector3(0,0,1).cross(axis).normalized();if(x.length()<0.1)x=Vector3(1,0,0);Vector3 z=x.cross(axis).normalized();
  manifold::mat3x4 matrix{{x.x,x.y,x.z},{axis.x,axis.y,axis.z},{z.x,z.y,z.z},{0,0,0}};
  auto piece=[&](double low,double high,double r0,double r1){configured_parts.push_back(manifold::Manifold::Cylinder(high-low,r1,r0,n).Rotate(90,0,0).Translate({0,high,0}).Transform(matrix));};
  if(type=="ball.mill"){
   auto ball=manifold::Manifold::Sphere(r,n).Translate({0,r,0})+manifold::Manifold::Cylinder(L-r,r,r,n).Rotate(-90,0,0).Translate({0,r,0});configured_parts.push_back(ball.Transform(matrix));
  }else if(type=="std.drill"){
   double h=r/std::tan(double(d["pointAngle"])*3.141592653589793/360);piece(0,h,0,r);piece(h,L,r,r);
  }else if(type=="center.drill"){
   double tipr=double(d["tipDiameter"])/2, h=tipr/std::tan(double(d["pointAngle"])*3.141592653589793/360), tl=double(d["tipLength"]);
   double shoulder=tl+(r-tipr)/std::tan(double(d["includedAngle"])*3.141592653589793/360);
   piece(0,h,0,tipr);piece(h,tl,tipr,tipr);piece(tl,shoulder,tipr,r);piece(shoulder,L,r,r);
  }else piece(0,L,r,r);
  for(auto &part:configured_parts){
   auto mesh=part.GetMeshGL64();std::vector<manifold::vec3> vertices;
   for(size_t i=0;i<mesh.vertProperties.size();i+=mesh.numProp)vertices.push_back({mesh.vertProperties[i],mesh.vertProperties[i+1],mesh.vertProperties[i+2]});
   configured_vertices.push_back(std::move(vertices));
  }
 }
 Ref<ArrayMesh> get_configured_tool()const{return to_mesh(manifold::Manifold::BatchBoolean(configured_parts,manifold::OpType::Add));}
 void begin_viewer_segment(){viewer_baseline=solid;viewer_points.clear();sweep_prefixes.clear();viewer_valid=true;}
 void reset_stock(Vector3 size,Vector3 center) {
  viewer_valid=false;
  air_cells.clear();solid=manifold::Manifold::Cube({size.x,size.y,size.z},true).Translate({center.x,center.y,center.z});last_ms=0;
 }
 void build_air_grid(){
  if(!air_grid_enabled || !air_cells.empty() || viewer_baseline.IsEmpty())return;
  // Include the full current contour baseline, even after a mid-contour
  // switch from fast mode, because final reconciliation may revisit it.
  auto box=viewer_baseline.BoundingBox();air_model_low=box.min;air_model_high=box.max;
  air_cell=std::max({box.max.x-box.min.x,box.max.y-box.min.y,box.max.z-box.min.z})/64.;
  if(!std::isfinite(air_cell) || air_cell<=0)return;
  air_projection_pad=std::max(viewer_baseline.GetEpsilon()*16.,air_cell*1e-9);
  air_origin=box.min-manifold::vec3{2*air_cell,2*air_cell,2*air_cell};
  air_nx=int(std::ceil((box.max.x-box.min.x)/air_cell))+4;air_ny=int(std::ceil((box.max.y-box.min.y)/air_cell))+4;air_nz=int(std::ceil((box.max.z-box.min.z)/air_cell))+4;
  air_cells.assign(air_nx*air_ny*air_nz,0);++air_grid_builds;
  auto mesh=viewer_baseline.GetMeshGL64();air_surfaces.clear();air_nodes.clear();air_triangles.clear();air_surfaces.reserve(mesh.triVerts.size()/3);
  for(size_t i=0;i<mesh.triVerts.size();i+=3){
   AirBox surface{{INFINITY,INFINITY,INFINITY},{-INFINITY,-INFINITY,-INFINITY}};
   for(int j=0;j<3;++j){auto k=mesh.triVerts[i+j]*mesh.numProp;for(int a=0;a<3;++a){surface.low[a]=std::min(surface.low[a],mesh.vertProperties[k+a]);surface.high[a]=std::max(surface.high[a],mesh.vertProperties[k+a]);}}
   AirTriangle triangle;for(int j=0;j<3;++j){auto k=mesh.triVerts[i+j]*mesh.numProp;triangle.p[j]={mesh.vertProperties[k],mesh.vertProperties[k+1],mesh.vertProperties[k+2]};}
   surface.triangle=air_triangles.size();air_triangles.push_back(triangle);air_surfaces.push_back(surface);
   int low[3]={air_nx-1,air_ny-1,air_nz-1},high[3]={0,0,0};int dims[3]={air_nx,air_ny,air_nz};
   for(int j=0;j<3;++j){auto k=mesh.triVerts[i+j]*mesh.numProp;for(int a=0;a<3;++a){double t=(mesh.vertProperties[k+a]-air_origin[a])/air_cell;low[a]=std::min(low[a],std::clamp(int(std::floor(t-1e-5)),0,dims[a]-1));high[a]=std::max(high[a],std::clamp(int(std::floor(t+1e-5)),0,dims[a]-1));}}
   // Triangle AABBs deliberately overfill surface cells. Never sample centers:
   // missing a thin surface could turn an interior cell into false air.
   for(int z=low[2];z<=high[2];++z)for(int y=low[1];y<=high[1];++y)for(int x=low[0];x<=high[0];++x)air_cells[(z*air_ny+y)*air_nx+x]=1;
  }
  air_nodes.reserve(air_surfaces.size()*2);if(!air_surfaces.empty())build_air_bvh(0,air_surfaces.size());
  std::vector<int> queue;queue.reserve(air_cells.size());
  auto visit=[&](int index){if(air_cells[index]==0){air_cells[index]=2;queue.push_back(index);}};
  for(int z=0;z<air_nz;++z)for(int y=0;y<air_ny;++y){visit((z*air_ny+y)*air_nx);visit((z*air_ny+y)*air_nx+air_nx-1);}
  for(int z=0;z<air_nz;++z)for(int x=0;x<air_nx;++x){visit(z*air_ny*air_nx+x);visit((z*air_ny+air_ny-1)*air_nx+x);}
  for(int y=0;y<air_ny;++y)for(int x=0;x<air_nx;++x){visit(y*air_nx+x);visit(((air_nz-1)*air_ny+y)*air_nx+x);}
  for(size_t i=0;i<queue.size();++i){int index=queue[i],x=index%air_nx,y=(index/air_nx)%air_ny,z=index/(air_nx*air_ny);
   if(x>0)visit(index-1);if(x+1<air_nx)visit(index+1);if(y>0)visit(index-air_nx);if(y+1<air_ny)visit(index+air_nx);if(z>0)visit(index-air_nx*air_ny);if(z+1<air_nz)visit(index+air_nx*air_ny);
  }
 }
 int build_air_bvh(int start,int end){
  AirNode node;node.start=start;node.count=end-start;node.box=air_surfaces[start];
  for(int i=start+1;i<end;++i)for(int a=0;a<3;++a){node.box.low[a]=std::min(node.box.low[a],air_surfaces[i].low[a]);node.box.high[a]=std::max(node.box.high[a],air_surfaces[i].high[a]);}
  int index=air_nodes.size();air_nodes.push_back(node);
  if(end-start>8){
   int axis=0;for(int a=1;a<3;++a)if(node.box.high[a]-node.box.low[a]>node.box.high[axis]-node.box.low[axis])axis=a;
   int middle=(start+end)/2;
   std::nth_element(air_surfaces.begin()+start,air_surfaces.begin()+middle,air_surfaces.begin()+end,[&](const AirBox&a,const AirBox&b){return a.low[axis]+a.high[axis]<b.low[axis]+b.high[axis];});
   int left=build_air_bvh(start,middle),right=build_air_bvh(middle,end);air_nodes[index].left=left;air_nodes[index].right=right;
  }
  return index;
 }
 bool air_box_overlap(const AirBox &box,manifold::vec3 low,manifold::vec3 high)const{
  double pad=air_cell*1e-5;
  for(int a=0;a<3;++a)if(high[a]<box.low[a]-pad || low[a]>box.high[a]+pad)return false;
  return true;
 }
 bool air_bvh_overlap(int index,manifold::vec3 low,manifold::vec3 high)const{
  auto &node=air_nodes[index];if(!air_box_overlap(node.box,low,high))return false;
  if(node.left>=0)return air_bvh_overlap(node.left,low,high) || air_bvh_overlap(node.right,low,high);
  for(int i=node.start;i<node.start+node.count;++i)if(air_box_overlap(air_surfaces[i],low,high))return true;
  return false;
 }
 double air_dot(manifold::vec3 a,manifold::vec3 b)const{return a.x*b.x+a.y*b.y+a.z*b.z;}
 manifold::vec3 air_cross(manifold::vec3 a,manifold::vec3 b)const{return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
 bool air_point_exterior(manifold::vec3 p)const{
  int cell[3],dims[3]={air_nx,air_ny,air_nz};
  for(int a=0;a<3;++a){if(p[a]<air_model_low[a]-air_cell*1e-5 || p[a]>air_model_high[a]+air_cell*1e-5)return true;cell[a]=int(std::floor((p[a]-air_origin[a])/air_cell));if(cell[a]<0 || cell[a]>=dims[a])return true;}
  return air_cells[(cell[2]*air_ny+cell[1])*air_nx+cell[0]]==2;
 }
 bool air_convex_overlap(int index,manifold::vec3 low,manifold::vec3 high,const std::vector<AirPlane>&planes,const std::vector<manifold::vec3>&vertices)const{
  auto &node=air_nodes[index];if(!air_box_overlap(node.box,low,high))return false;
  if(node.left>=0)return air_convex_overlap(node.left,low,high,planes,vertices) || air_convex_overlap(node.right,low,high,planes,vertices);
  double pad=air_projection_pad;
  for(int i=node.start;i<node.start+node.count;++i){
   auto &box=air_surfaces[i];if(!air_box_overlap(box,low,high))continue;auto &triangle=air_triangles[box.triangle];bool separated=false;
   for(auto &plane:planes){if(air_dot(plane.normal,triangle.p[0])-plane.offset>pad && air_dot(plane.normal,triangle.p[1])-plane.offset>pad && air_dot(plane.normal,triangle.p[2])-plane.offset>pad){separated=true;break;}}
   if(separated)continue;
   auto normal=air_cross(triangle.p[1]-triangle.p[0],triangle.p[2]-triangle.p[0]);double length=std::sqrt(air_dot(normal,normal));
   if(length>1e-20){normal=normal/length;double triangle_min=INFINITY,triangle_max=-INFINITY,min_distance=INFINITY,max_distance=-INFINITY;for(auto &p:triangle.p){double distance=air_dot(normal,p);triangle_min=std::min(triangle_min,distance);triangle_max=std::max(triangle_max,distance);}for(auto &p:vertices){double distance=air_dot(normal,p);min_distance=std::min(min_distance,distance);max_distance=std::max(max_distance,distance);}if(min_distance>triangle_max+pad || max_distance<triangle_min-pad)continue;}
   // No separating face plane: keep the exact boolean. We deliberately omit
   // edge-cross axes, accepting false positives rather than missing material.
   return true;
  }
  return false;
 }
 bool convex_may_have_material(const manifold::Manifold &tool,manifold::vec3 low,manifold::vec3 high)const{
  if(!air_grid_enabled || air_cells.empty() || air_nodes.empty())return true;
  auto mesh=tool.GetMeshGL64();if(mesh.triVerts.size()>3000)return true;
  std::vector<manifold::vec3> vertices;vertices.reserve(mesh.vertProperties.size()/mesh.numProp);bool exterior=false;
  for(size_t i=0;i<mesh.vertProperties.size();i+=mesh.numProp){manifold::vec3 p{mesh.vertProperties[i],mesh.vertProperties[i+1],mesh.vertProperties[i+2]};vertices.push_back(p);if(air_point_exterior(p))exterior=true;}
  if(!exterior)return true;
  std::vector<AirPlane> planes;
  for(size_t i=0;i<mesh.triVerts.size();i+=3){auto p=vertices[mesh.triVerts[i]],normal=air_cross(vertices[mesh.triVerts[i+1]]-p,vertices[mesh.triVerts[i+2]]-p);double length=std::sqrt(air_dot(normal,normal));if(length<1e-20)continue;normal=normal/length;bool duplicate=false;for(auto &plane:planes)if(air_dot(normal,plane.normal)>1.-1e-12){duplicate=true;break;}if(!duplicate){double offset=-INFINITY;for(auto &vertex:vertices)offset=std::max(offset,air_dot(normal,vertex));planes.push_back({normal,offset});}}
  return planes.empty() || air_convex_overlap(0,low,high,planes,vertices);
 }
 bool may_have_material(manifold::vec3 low,manifold::vec3 high)const{
  if(!air_grid_enabled || air_cells.empty())return true;
  int lo[3],hi[3],dims[3]={air_nx,air_ny,air_nz};
  for(int a=0;a<3;++a){lo[a]=int(std::floor((low[a]-air_origin[a])/air_cell-1e-5));hi[a]=int(std::floor((high[a]-air_origin[a])/air_cell+1e-5));if(hi[a]<0 || lo[a]>=dims[a])return false;lo[a]=std::max(0,lo[a]);hi[a]=std::min(dims[a]-1,hi[a]);}
  bool all_air=true;
  for(int z=lo[2];z<=hi[2] && all_air;++z)for(int y=lo[1];y<=hi[1] && all_air;++y)for(int x=lo[0];x<=hi[0];++x)if(air_cells[(z*air_ny+y)*air_nx+x]!=2){all_air=false;break;}
  if(all_air)return false;
  // If a connected query box contains an exterior witness and no surface,
  // the whole box is exterior. The BVH resolves clearances smaller than a cell.
  bool exterior=false;
  for(int a=0;a<3;++a)if(low[a]<air_model_low[a]-air_cell*1e-5 || high[a]>air_model_high[a]+air_cell*1e-5)exterior=true;
  for(int corner=0;corner<8 && !exterior;++corner){int cell[3];bool inside=true;for(int a=0;a<3;++a){double p=corner&(1<<a)?high[a]:low[a];cell[a]=int(std::floor((p-air_origin[a])/air_cell));if(cell[a]<0 || cell[a]>=dims[a])inside=false;}if(inside && air_cells[(cell[2]*air_ny+cell[1])*air_nx+cell[0]]==2)exterior=true;}
  if(exterior && !air_nodes.empty() && !air_bvh_overlap(0,low,high))return false;
  return true;
 }
 int cut_viewer(const PackedVector3Array &batch,double radius,double length,bool ball,double tolerance) {
  return cut_viewer_impl(batch,radius,length,ball,tolerance,nullptr);
 }
 int cut_viewer_rows(const Array &rows,double radius,double length,bool ball,double tolerance) {
  if(rows.is_empty() || rows.size()>16 || !std::isfinite(radius) || !std::isfinite(length) || !std::isfinite(tolerance) || radius<=0 || length<=0 || tolerance<=0)return -1;
  size_t total=0;
  for(int i=0;i<rows.size();++i){
   if(rows[i].get_type()!=Variant::PACKED_VECTOR3_ARRAY)return -1;
   PackedVector3Array row=rows[i];if(row.size()<2)return -1;
   total+=row.size();if(total>262144)return -1;
   for(int j=0;j<row.size();++j)if(!std::isfinite(row[j].x) || !std::isfinite(row[j].y) || !std::isfinite(row[j].z))return -1;
  }
  auto started=std::chrono::steady_clock::now();
  auto baseline=solid;
  bool saved_incremental=incremental_stream_cut,saved_cache=stream_sweep_cache;
  incremental_stream_cut=true;stream_sweep_cache=false;
  std::vector<manifold::Manifold> sweeps;
  for(int i=0;i<rows.size();++i){
   PackedVector3Array row=rows[i];
   // Reset at every row: never build a cutting chord between separate contours.
   begin_viewer_segment();
   for(int offset=0;offset<row.size()-1;offset+=64)
    cut_viewer_impl(row.slice(offset,std::min(offset+65,int(row.size()))),radius,length,ball,tolerance,&sweeps);
  }
  auto begin=std::chrono::steady_clock::now();
  int status=0;
  if(!sweeps.empty()){
   auto combined=manifold::Manifold::BatchBoolean(sweeps,manifold::OpType::Add);
   auto next=baseline-combined;status=static_cast<int>(next.Status());
   if(status==0){solid=next;last_sweep=combined;}
  }else last_sweep=manifold::Manifold();
  boolean_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  incremental_stream_cut=saved_incremental;stream_sweep_cache=saved_cache;
  viewer_valid=false;viewer_points.clear();air_cells.clear();
  last_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  return status;
 }
 int cut_viewer_impl(const PackedVector3Array &batch,double radius,double length,bool ball,double tolerance,std::vector<manifold::Manifold>* deferred) {
  auto begin=std::chrono::steady_clock::now();
  int result_status=0;
  if(!viewer_valid)begin_viewer_segment();
  if(incremental_stream_cut || stream_sweep_cache)build_air_grid();
  if(stream_sweep_cache && (cached_radius!=radius || cached_length!=length || cached_tolerance!=tolerance || cached_ball!=ball)){
   sweep_prefixes.clear();cached_radius=radius;cached_length=length;cached_tolerance=tolerance;cached_ball=ball;
  }
  for(int i=0;i<batch.size();++i){
   auto p=batch[i];int n=viewer_points.size();
   if(n && p.distance_to(viewer_points[n-1])<1e-7)continue;
   if(n>=2){auto a=viewer_points[n-1]-viewer_points[n-2];auto b=p-viewer_points[n-1];if(a.cross(b).length()<1e-6 && a.dot(b)>0)viewer_points.resize(n-1);}
   viewer_points.push_back(p);
  }
  const bool use_prefix_cache=stream_sweep_cache && !incremental_stream_cut;
  auto cut_baseline=incremental_stream_cut ? solid : viewer_baseline;
  PackedVector3Array points=incremental_stream_cut ? batch : viewer_points;
  input_points+=points.size();
  // Remove float-scale redundant points only from transient delta previews.
  // The canonical contour still uses its full original path.
  double preview_error=incremental_stream_cut ? 1e-7 : 0.;
  double path_error=std::max(compression_error,preview_error);
  if(path_error>0 && points.size()>2){
   std::vector<bool> keep(points.size(),false);keep.front()=keep.back()=true;
   std::vector<std::pair<int,int>> stack{{0,int(points.size()-1)}};
   while(!stack.empty()){
    auto range=stack.back();stack.pop_back();auto a=points[range.first],b=points[range.second];auto delta=b-a;double length2=delta.length_squared(),max_distance=path_error;int split=-1;
    for(int i=range.first+1;i<range.second;++i){double t=length2>1e-15?std::clamp(double((points[i]-a).dot(delta))/length2,0.0,1.0):0.0;double distance=points[i].distance_to(a+delta*t);if(distance>max_distance){max_distance=distance;split=i;}}
    if(split>=0){keep[split]=true;stack.push_back({range.first,split});stack.push_back({split,range.second});}
   }
   auto uncompressed=points;points.clear();for(int i=0;i<uncompressed.size();++i)if(keep[i])points.push_back(uncompressed[i]);
  }
  kept_points+=points.size();
  auto hull_start=std::chrono::steady_clock::now();
  int segments=std::min(256,std::max(24,int(std::ceil(3.141592653589793/std::acos(std::max(-1.0,1.0-tolerance/radius))))));
  segments=(segments+3)/4*4;
  auto parts=configured_parts;
  if(parts.empty()){
   auto base=manifold::Manifold::Cylinder(length-(ball?radius:0),radius,radius,segments).Rotate(-90,0,0).Translate({0,ball?radius:0,0});
   if(ball)base=base+manifold::Manifold::Sphere(radius,segments).Translate({0,radius,0});
   parts.push_back(base);
  }
  if(use_prefix_cache)sweep_prefixes.resize(parts.size());
  auto stock_box=cut_baseline.BoundingBox();
  std::vector<manifold::Manifold> sweeps;
  for(size_t part_index=0;part_index<parts.size();++part_index){
   auto &part=parts[part_index];
   std::vector<manifold::vec3> fallback_vertices;
   if(configured_vertices.empty()){
    auto mesh=part.GetMeshGL64();for(size_t i=0;i<mesh.vertProperties.size();i+=mesh.numProp)fallback_vertices.push_back({mesh.vertProperties[i],mesh.vertProperties[i+1],mesh.vertProperties[i+2]});
   }
   const auto &vertices=configured_vertices.empty()?fallback_vertices:configured_vertices[part_index];
   std::vector<manifold::vec3> cloud;cloud.reserve(2*vertices.size());
   auto bounds=part.BoundingBox();
   int first=points.size()>1?1:0;
   manifold::Manifold prefix;bool has_prefix=false;
   if(use_prefix_cache){
    auto &cache=sweep_prefixes[part_index];bool matches=cache.points.size()<=points.size();
    for(int i=0;matches && i<cache.points.size();++i)matches=cache.points[i]==points[i];
    if(matches && cache.points.size()>1){prefix=cache.solid;has_prefix=cache.has;first=cache.points.size();++prefix_hits;}
    else cache=SweepPrefix();
   }
   auto save_prefix=[&](int i){
    if(use_prefix_cache && i<points.size()-1){auto &cache=sweep_prefixes[part_index];cache.solid=prefix;cache.has=has_prefix;cache.points=points.slice(0,i+1);}
   };
   for(int i=first;i<points.size();++i){
    auto p=points[i],q=points[i>0?i-1:i];
    manifold::vec3 low{std::min(p.x,q.x)+bounds.min.x,std::min(p.y,q.y)+bounds.min.y,std::min(p.z,q.z)+bounds.min.z};
    manifold::vec3 high{std::max(p.x,q.x)+bounds.max.x,std::max(p.y,q.y)+bounds.max.y,std::max(p.z,q.z)+bounds.max.z};
    if(high.x<stock_box.min.x || high.y<stock_box.min.y || high.z<stock_box.min.z || low.x>stock_box.max.x || low.y>stock_box.max.y || low.z>stock_box.max.z){save_prefix(i);continue;}
    if(!may_have_material(low,high)){++air_skipped;save_prefix(i);continue;}
    auto tool=part.Translate({p.x,p.y,p.z});
    if(i>0 && p.distance_to(q)>1e-7){
     cloud.clear();for(auto &v:vertices)cloud.push_back(v+manifold::vec3{p.x,p.y,p.z});for(auto &v:vertices)cloud.push_back(v+manifold::vec3{q.x,q.y,q.z});
     tool=manifold::Manifold::Hull(cloud);
    }
    if(i>0 && p.distance_to(q)>1e-7 && !convex_may_have_material(tool,low,high)){++air_skipped;++air_convex_skipped;save_prefix(i);continue;}
    ++sweep_count;
    if(use_prefix_cache){prefix=has_prefix ? prefix+tool : tool;has_prefix=true;save_prefix(i);}
    else sweeps.push_back(tool);
   }
   if(use_prefix_cache && has_prefix)sweeps.push_back(prefix);
  }
  hull_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-hull_start).count();
  if(deferred){
   deferred->insert(deferred->end(),sweeps.begin(),sweeps.end());
   last_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
   return 0;
  }
  auto boolean_start=std::chrono::steady_clock::now();
  if(sweeps.empty()){solid=cut_baseline;last_sweep=manifold::Manifold();}
  if(!sweeps.empty()) {
   last_sweep=manifold::Manifold::BatchBoolean(sweeps,manifold::OpType::Add);
   auto next=cut_baseline-last_sweep;
   result_status=static_cast<int>(next.Status());
   // Cutter tolerance controls tessellation, not cumulative stock erosion.
   // Repeated Simplify(tolerance) across hundreds of rows can tilt a flat
   // wall into visible narrow grooves, even when each local change is small.
   if(result_status==0)solid=next;
  }
  last_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  boolean_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-boolean_start).count();
  return result_status;
 }
 int finish_stream_segment(double radius,double length,bool ball,double tolerance){
  // Reconcile once per contour against its original stock and complete path.
  // This prevents numerical drift without simplifying or moving the surface.
  bool saved_incremental=incremental_stream_cut,saved_cache=stream_sweep_cache;
  incremental_stream_cut=false;stream_sweep_cache=false;
  int result=cut_viewer(PackedVector3Array(),radius,length,ball,tolerance);
  incremental_stream_cut=saved_incremental;stream_sweep_cache=saved_cache;
  return result;
 }
 void reset() { air_cells.clear();viewer_valid=false;solid = manifold::Manifold::Cube({7.2,0.8,4.8}).Translate({-3.6,0,-2.4}); last_ms=0; }
 void reset_plate() { air_cells.clear();viewer_valid=false;solid=manifold::Manifold::Cube({7.2,0.2,4.8}).Translate({-3.6,0,-2.4});last_ms=0; }
 void set_target(const PackedVector3Array &vertices,const PackedInt32Array &indices) {
  manifold::MeshGL mesh;
  for(int i=0;i<vertices.size();++i) { mesh.vertProperties.push_back(vertices[i].x);mesh.vertProperties.push_back(vertices[i].y);mesh.vertProperties.push_back(vertices[i].z); }
  for(int i=0;i<indices.size();++i) mesh.triVerts.push_back(indices[i]);
  target=manifold::Manifold(mesh);
 }
 Dictionary inspect_target() const {
  Dictionary d;d["target_volume"]=target.Volume();d["target_status"]=static_cast<int>(target.Status());d["missing"]=(target-solid).Volume();d["excess"]=(solid-target).Volume();return d;
 }
 // Inspect the float32 geometry delivered to Godot. Reconstruct numerical
 // provenance so accumulated machining tolerances cannot simplify away thin
 // differences. The live machining state remains unchanged.
 manifold::Manifold reference_body(const manifold::Manifold &body) const {
  auto mesh=body.GetMeshGL();mesh.tolerance=0;
  return manifold::Manifold(mesh);
 }
 // Intermediate subtract-only checkpoints need target loss, not excess stock.
 // Keep the same float reference reconstruction as the full final audit.
 // This is deliberately not a volume-balance or final-product certification.
 Dictionary inspect_target_reference_missing() const {
  auto a=reference_body(solid),b=reference_body(target);
  auto missing=b-a;
  double mv=missing.Volume(),tv=b.Volume();
  bool valid=a.Status()==manifold::Manifold::Error::NoError && b.Status()==manifold::Manifold::Error::NoError && missing.Status()==manifold::Manifold::Error::NoError && std::isfinite(mv) && std::isfinite(tv) && tv>0 && mv>=-1e-8;
  Dictionary d;d["missing_check_valid"]=valid;d["missing"]=mv;d["target_volume"]=tv;return d;
 }
 Dictionary inspect_target_reference() const {
  auto a=reference_body(solid),b=reference_body(target);
  auto missing=b-a,excess=a-b;
  double mv=missing.Volume(),ev=excess.Volume(),tv=b.Volume(),sv=a.Volume();
  double balance=std::abs((ev-mv)-(sv-tv));
  bool valid=a.Status()==manifold::Manifold::Error::NoError && b.Status()==manifold::Manifold::Error::NoError && missing.Status()==manifold::Manifold::Error::NoError && excess.Status()==manifold::Manifold::Error::NoError && mv>=-1e-8 && ev>=-1e-8 && balance<=std::max(1e-7,tv*1e-8);
  Dictionary d;d["target_volume"]=tv;d["stock_volume"]=sv;d["target_status"]=int(b.Status());d["missing"]=mv;d["excess"]=ev;d["comparison_valid"]=valid;d["volume_balance_error"]=balance;d["live_stock_epsilon"]=solid.GetEpsilon();d["live_stock_tolerance"]=solid.GetTolerance();return d;
 }
 Ref<ArrayMesh> get_reference_difference_mesh(bool missing)const {
  auto a=reference_body(solid),b=reference_body(target);
  return to_mesh(missing ? b-a : a-b);
 }
 Ref<ArrayMesh> get_difference_mesh(bool missing)const{return to_mesh(missing ? target-solid : solid-target);}
 double excess_in_mesh(const PackedVector3Array &v,const PackedInt32Array &idx)const{
  manifold::MeshGL mesh;
  for(int i=0;i<v.size();++i){mesh.vertProperties.push_back(v[i].x);mesh.vertProperties.push_back(v[i].y);mesh.vertProperties.push_back(v[i].z);}
  for(int i=0;i<idx.size();++i)mesh.triVerts.push_back(idx[i]);
  mesh.Merge();auto mask=manifold::Manifold(mesh);
  if(mask.Status()!=manifold::Manifold::Error::NoError)return -1;
  return ((solid-target)^mask).Volume();
 }
 double material_in_box(Vector3 minimum,Vector3 maximum) const {
  auto size=maximum-minimum;
  return (solid ^ manifold::Manifold::Cube({size.x,size.y,size.z}).Translate({minimum.x,minimum.y,minimum.z})).Volume();
 }
 // A segment swept by a vertical flat-end mill is the hull of its endpoint cylinders.
 // Layers stay at constant depth; vertical transitions are safely included in the hull.
 void cut_batch(const PackedVector3Array &points, double radius) {
  auto begin = std::chrono::steady_clock::now();
  std::vector<manifold::Manifold> tools;
  auto cylinder_at = [&](Vector3 p) {
   return manifold::Manifold::Cylinder(3.0,radius,radius,96)
    .Rotate(-90,0,0).Translate({p.x,p.y-0.002,p.z});
  };
  for (int i=0;i<points.size();++i) {
   if(points[i].y>0.81) continue;
   if(i>0 && points[i].distance_to(points[i-1]) < 0.000001) continue;
   auto tool = cylinder_at(points[i]);
   if(i>0 && points[i-1].y<=0.81 && std::abs(points[i].y-points[i-1].y)<0.000001 && points[i].distance_to(points[i-1]) < 0.4)
    tool = manifold::Manifold::Hull(std::vector<manifold::Manifold>{tool,cylinder_at(points[i-1])});
   tools.push_back(tool);
  }
  if(!tools.empty()) {
   auto next = solid - manifold::Manifold::BatchBoolean(tools,manifold::OpType::Add);
   if(next.Status()==manifold::Manifold::Error::NoError) solid=next;
  }
  last_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
 }
 Ref<ArrayMesh> get_mesh() {auto start=std::chrono::steady_clock::now();auto out=to_mesh(solid);export_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();return out;}
 Ref<ArrayMesh> get_display_mesh()const{return to_mesh(solid,true);}
 Ref<ArrayMesh> get_last_sweep() const {return to_mesh(last_sweep);}
 Array get_mesh_arrays() const {return mesh_arrays(solid);}
 Array get_compact_mesh_arrays() const {
  auto mesh=solid.GetMeshGL();
  struct Corner {Vector3 normal;int index,next;};
  std::vector<int> heads(mesh.vertProperties.size()/mesh.numProp,-1);
  std::vector<Corner> shared;shared.reserve(mesh.triVerts.size());
  PackedVector3Array vertices,normals;PackedInt32Array indices;
  vertices.resize(mesh.triVerts.size());normals.resize(mesh.triVerts.size());indices.resize(mesh.triVerts.size());
  int count=0,cursor=0;
  for(size_t i=0;i<mesh.triVerts.size();i+=3){
   Vector3 p[3];for(int j=0;j<3;++j){auto k=mesh.triVerts[i+j]*mesh.numProp;p[j]=Vector3(mesh.vertProperties[k],mesh.vertProperties[k+1],mesh.vertProperties[k+2]);}
   auto normal=(p[1]-p[0]).cross(p[2]-p[0]).normalized();
   for(int j:{0,2,1}){
    int &head=heads[mesh.triVerts[i+j]];int index=-1;
    // Share only identical positions AND normals, preserving every sharp edge.
    for(int k=head;k>=0;k=shared[k].next)if(shared[k].normal==normal){index=shared[k].index;break;}
    if(index<0){index=count++;vertices.set(index,p[j]);normals.set(index,normal);shared.push_back({normal,index,head});head=shared.size()-1;}
    indices.set(cursor++,index);
   }
  }
  vertices.resize(count);normals.resize(count);
  Array arrays;arrays.resize(Mesh::ARRAY_MAX);arrays[Mesh::ARRAY_VERTEX]=vertices;arrays[Mesh::ARRAY_NORMAL]=normals;arrays[Mesh::ARRAY_INDEX]=indices;return arrays;
 }
 Ref<ArrayMesh> to_mesh(const manifold::Manifold &body,bool smooth=false) const {
  auto arrays=mesh_arrays(body,smooth);
  Ref<ArrayMesh> out;out.instantiate();
  PackedVector3Array vertices=arrays[Mesh::ARRAY_VERTEX];
  if(vertices.size())out->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES,arrays);
  return out;
 }
 Array mesh_arrays(const manifold::Manifold &body,bool smooth=false) const {
  auto mesh = body.GetMeshGL();
  PackedVector3Array vertices, normals;
  std::vector<Vector3> face_normals;
  std::vector<std::vector<std::pair<size_t,double>>> adjacent;
  if(smooth){
   adjacent.resize(mesh.vertProperties.size()/mesh.numProp);
   for(size_t i=0;i<mesh.triVerts.size();i+=3){
    Vector3 p[3];for(int j=0;j<3;++j){auto k=mesh.triVerts[i+j]*mesh.numProp;p[j]=Vector3(mesh.vertProperties[k],mesh.vertProperties[k+1],mesh.vertProperties[k+2]);}
    face_normals.push_back((p[1]-p[0]).cross(p[2]-p[0]).normalized());
    for(int j=0;j<3;++j){auto a=(p[(j+1)%3]-p[j]).normalized(),b=(p[(j+2)%3]-p[j]).normalized();double angle=std::acos(std::clamp(double(a.dot(b)),-1.0,1.0));adjacent[mesh.triVerts[i+j]].push_back({i/3,angle});}
   }
  }
  // Duplicate vertices per face to keep planar surfaces flat instead of smoothing sharp edges.
  for(size_t i=0;i<mesh.triVerts.size();i+=3) {
   Vector3 p[3];
   for(int j=0;j<3;++j) { auto k=mesh.triVerts[i+j]*mesh.numProp; p[j]=Vector3(mesh.vertProperties[k],mesh.vertProperties[k+1],mesh.vertProperties[k+2]); }
   Vector3 n=(p[1]-p[0]).cross(p[2]-p[0]).normalized();
   for(int j : {0,2,1}) {
    Vector3 display_normal=n;
    if(smooth){Vector3 sum;for(auto &entry:adjacent[mesh.triVerts[i+j]])if(n.dot(face_normals[entry.first])>0.9063078)sum+=face_normals[entry.first]*entry.second;if(sum.length_squared()>1e-12)display_normal=sum.normalized();}
    vertices.push_back(p[j]);normals.push_back(display_normal);
   }
  }
  Array arrays;arrays.resize(Mesh::ARRAY_MAX);arrays[Mesh::ARRAY_VERTEX]=vertices;arrays[Mesh::ARRAY_NORMAL]=normals;
  return arrays;
 }
 Array get_stock_components() const {
  Array result;
  for(const auto &part:solid.Decompose()){
   Dictionary item;item["mesh"]=to_mesh(part,true);item["volume"]=part.Volume();
   item["status"]=static_cast<int>(part.Status());result.push_back(item);
  }
  return result;
 }
 bool retain_at(Vector3 point) {
  auto probe = manifold::Manifold::Cube({0.002,0.002,0.002}).Translate({point.x-0.001,point.y-0.001,point.z-0.001});
  auto parts=solid.Decompose();
  if(parts.size()<2) return false;
  for(auto &part : parts) {
   if((part ^ probe).Volume()>0.000000001) { solid=part; return true; }
  }
  return false;
 }
 Dictionary inspect() const {
  auto cyl=[](double x,double z,double radius,double bottom) {return manifold::Manifold::Cylinder(3,radius,radius,96).Rotate(-90,0,0).Translate({x,bottom,z});};
  auto holes=cyl(-2.28,0,0.6,-0.01)+cyl(2.28,0,0.6,-0.01);
  std::vector<manifold::Manifold> corners;
  for(double x : {-1.25,1.25}) for(double z : {-0.6,0.6}) corners.push_back(cyl(x,z,0.24,0.398));
  auto target=manifold::Manifold::Cube({7.2,0.8,4.8}).Translate({-3.6,0,-2.4}) - holes - manifold::Manifold::Hull(corners);
  double missing=(target-solid).Volume(), excess=(solid-target).Volume();
  auto pins=cyl(-2.28,0,0.49,-0.01)+cyl(2.28,0,0.49,-0.01);
  double collision=(solid ^ pins).Volume();
  // An oversized hole can fit a pin, but cannot locate it securely.
  auto required_wall=(cyl(-2.28,0,0.74,0.02)+cyl(2.28,0,0.74,0.02))-holes;
  required_wall = required_wall ^ manifold::Manifold::Cube({7.2,0.8,4.8}).Translate({-3.6,0,-2.4});
  double wall_loss=(required_wall-solid).Volume();
  Dictionary result;
  result["score"]=std::max(0.0,100.0-400.0*(missing+excess)/27.648);
  result["fits"]=collision<0.00001 && wall_loss<0.01;
  result["collision_volume"]=collision;
  result["error_volume"]=missing+excess;
  return result;
 }
 double volume() const {return solid.Volume();}
 double last_milliseconds() const {return last_ms;}
 int status() const {return static_cast<int>(solid.Status());}
};
void initialize(ModuleInitializationLevel level) { if(level==MODULE_INITIALIZATION_LEVEL_SCENE) {ClassDB::register_class<CNCManifold>();ClassDB::register_class<CNCSurfaceDeviation>();} }
void uninitialize(ModuleInitializationLevel) {}
extern "C" GDExtensionBool GDE_EXPORT cnc_manifold_init(GDExtensionInterfaceGetProcAddress proc,GDExtensionClassLibraryPtr lib,GDExtensionInitialization *init) {
 GDExtensionBinding::InitObject obj(proc,lib,init); obj.register_initializer(initialize); obj.register_terminator(uninitialize); obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE); return obj.init();
}
