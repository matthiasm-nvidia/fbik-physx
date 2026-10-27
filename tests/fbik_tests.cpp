#include "Fbik.h"
#include "PhysicsSimulation.h"
#include <ovstage/ovstage_config.h>
#include <ovphysx/ovphysx.h>
#include <ovphysx/experimental/OvStageOutput.hpp>
#include <filesystem>
#include <iostream>
#include <cmath>
#include <stdexcept>

namespace st = viewer::stage;
void require(bool value,const char *message) { if (!value) throw std::runtime_error(message); }
void check(ovphysx_result_t r)
{
    if (r.status!=OVPHYSX_API_SUCCESS) { const auto e=ovphysx_get_last_error();throw std::runtime_error(std::string(e.ptr,e.length)); }
}
struct Scene
{
    ovstage_instance_t *stage=nullptr;
    ovstage_ordinal_t ordinal=1;
    ~Scene() { if(stage) ovstage_destroy_instance(stage); }
    void load(const std::filesystem::path &file)
    {
        ovstage_instance_desc_t desc{};desc.name="fbik-test";
        st::check(ovstage_create_instance(&desc,&stage));
        ovstage_population_desc_t population{};population.domains=OVSTAGE_POPULATION_DOMAIN_ALL;
        ovstage_population_selector_t selector{};
        selector.prim_predicate.kind=OVSTAGE_POPULATION_PRIM_PREDICATE_ALL;
        std::vector<ovx_string_t> properties;
    for (const char *name : {"joints", "bindTransforms", "restTransforms", "translations", "rotations", "scales",
             "points", "skel:skeleton", "skel:animationSource", "skel:joints", "primvars:skel:geomBindTransform",
             "primvars:skel:jointIndices", "primvars:skel:jointWeights", "primvars:skel:jointIndices:indices",
             "primvars:skel:jointWeights:indices", "primvars:skel:skinningMethod", "skel:blendShapeTargets",
             "fbik:joint", "fbik:skeleton"}) properties.push_back(st::str(name));
        selector.property_predicate=ovstage_population_property_predicate_has_name(properties.data(), properties.size());
        const ovx_string_t metadata[]={st::str("interpolation"),st::str("elementSize")};
        selector.property_metadata_paths=metadata;selector.property_metadata_path_count=2;
        population.selectors=&selector;population.selector_count=1;
        const auto units=st::str("metersPerUnit");population.stage_metadata_paths=&units;population.stage_metadata_path_count=1;
        const auto path=file.u8string();
        auto op=ovstage_population_open_usd_from_file_with_desc(stage,st::str(path.c_str()),ordinal,0,&population,1);
        st::check(op.status);ovstage_population_op_wait_result_t result{};
        st::check(ovstage_population_wait_op(stage,op.op_index,OVSTAGE_TIMEOUT_INFINITE,&result));
        require(result.error_op_id_count==0,"Failed to populate fixture");
        st::seal(stage,ordinal);st::hierarchy(stage,ordinal);
    }
};
struct Physics
{
    ovphysx_handle_t handle=OVPHYSX_INVALID_HANDLE;
    ~Physics() { if(handle) {ovphysx_detach_ovstage(handle);ovphysx_destroy_instance(handle);} }
};
int main(int argc,char **argv)
{
    int code=0;bool initialized=false;
    try
    {
        require(argc==2,"Expected repository root");
        const auto root=std::filesystem::absolute(argv[1]);
        const auto binaries=(root/"deps/ovstage/bin").u8string();
        auto entry=ovstage_config_entry_binary_package_root_path(st::str(binaries.c_str()));
        ovstage_config_t config{&entry,1};st::check(ovstage_initialize(&config));initialized=true;
        viewer::PhysicsRuntime runtime;
        {
            Scene scene;scene.load(root/"tests/data/characters.usda");
            fbik::Module module;
            auto stats=module.init(scene.stage,scene.ordinal);
            require(stats.characters==2 && stats.skeletons==3,"Multiple characters/skeletons were not discovered");
            require(stats.boxes==1 && stats.capsules==3 && stats.leaves==6 && stats.degenerate==1,"Wrong OBB/capsule/leaf counts");
            auto first=st::read(scene.stage,scene.ordinal,{"usd-prim-type","omni:fabric:worldMatrix","fbik:joint","height","radius","physics:rigidBodyEnabled"});
            const std::string a="/World/A/__FbikBoneColliders/Skeleton_0/";
            require(first.at(a+"Joint_0").at("usd-prim-type").token=="Cube","Skin OBB was not a box");
            require(first.at(a+"Joint_1").at("usd-prim-type").token=="Capsule","Non-leaf with End in its name was skipped");
            require(!first.count(a+"Joint_2") && !first.count(a+"Joint_3") && !first.count(a+"Joint_4") && !first.count(a+"Joint_5"),"Leaf or degenerate segment has collider");
            const auto &capsule=first.at(a+"Joint_1");
            require(std::abs(capsule.at("height").numbers[0]-2)<1e-5 && std::abs(capsule.at("radius").numbers[0]-.2)<1e-5,"Fallback capsule geometry mismatch");
            require(std::abs(capsule.at("omni:fabric:worldMatrix").numbers[13]-4)<1e-5,"Character scale was not respected");
            const auto &branch=first.at("/World/B/__FbikBoneColliders/Skeleton_0/Joint_0");
            require(std::abs(branch.at("height").numbers[0]-2)<1e-5 &&
                    std::abs(branch.at("omni:fabric:worldMatrix").numbers[13]-1)<1e-5,
                    "Capsule does not point to the average child position");
            const auto &box=first.at(a+"Joint_0").at("omni:fabric:worldMatrix").numbers;
            require(std::abs(box[12]-3.7)<.001,"Skin remap, geomBind, multi-mesh union or character transform incorrect");
            for (const auto &[path,prim] : first)
                if(path.find("__FbikBoneColliders")!=std::string::npos)
                {
                    auto it=prim.find("physics:rigidBodyEnabled");
                    require(it==prim.end() || it->second.numbers.empty() || it->second.numbers[0]==0,"Collider is not static");
                }
            auto repeat=module.init(scene.stage,scene.ordinal);
            auto second=st::read(scene.stage,scene.ordinal,{"usd-prim-type","omni:fabric:worldMatrix","fbik:joint","height","radius","physics:rigidBodyEnabled"});
            require(repeat.boxes==stats.boxes && repeat.capsules==stats.capsules && second.size()==first.size(),"init created duplicates");
            module.showColliders(true,scene.ordinal);
            module.showColliders(false,scene.ordinal);
            Physics physics;ovphysx_create_args args=OVPHYSX_CREATE_ARGS_DEFAULT;
            check(ovphysx_create_instance(&args,&physics.handle));check(ovphysx_attach_ovstage(physics.handle,scene.stage,scene.ordinal));
            for(int i=0;i<60;++i) check(ovphysx_step_sync(physics.handle,1.0f/60));
            const float from[]={3.4f,0,3},direction[]={0,0,-1};
            const ovphysx_scene_query_hit_t *hits=nullptr;uint32_t count=0;
            check(ovphysx_raycast(physics.handle,from,direction,10,false,OVPHYSX_SCENE_QUERY_MODE_CLOSEST,&hits,&count));
            require(count==1,"Generated hidden box has no PhysX shape");
            ovphysx_string_t path{};const auto collider=hits[0].collision;
            check(ovphysx_scene_query_get_paths_from_ids(physics.handle,&collider,1,&path,1,&count));
            require(std::string(path.ptr,path.length).find("__FbikBoneColliders")!=std::string::npos,"Ray missed generated collider");
            auto output=ovphysx::utils::writeWorldTransformsToOvstage(physics.handle,scene.stage,++scene.ordinal);
            require(output.ok(),"Could not publish probe position");st::seal(scene.stage,scene.ordinal);
            auto state=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"});
            const auto y=state.at("/World/Probe").at("omni:fabric:worldMatrix").numbers[13];
            std::cout << "Probe contacting generated collider at y=" << y << '\n';
            require(y>.3 && y<2,"Dynamic probe did not collide with static bone box");
        }
        {
            Scene scene;scene.load(root/"resources/scenes/falling_box.usda");
            fbik::Module module;auto stats=module.init(scene.stage,scene.ordinal);
            require(stats.characters==1 && stats.skeletons==1 && stats.boxes==61 && stats.leaves==16,"SOMA collider count mismatch");
            const auto data=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix","fbik:joint"});
            for(const auto &[path,prim]:data)
                if(prim.count("fbik:joint"))
                {
                    const auto &m=prim.at("omni:fabric:worldMatrix").numbers;
                    require(m.size()==16 && m[12]>0 && m[12]<4 && m[13]>-1 && m[13]<3,"SOMA collider outside expected character bounds");
                }
            std::cout << "SOMA: " << stats.boxes << " boxes, " << stats.leaves << " leaves omitted\n";
        }
        std::cout << "FBIK discovery, bindings, OBBs, fallback capsules, static collision and repeated init passed.\n";
    }
    catch(const std::exception &e) { std::cerr << e.what() << '\n';code=1; }
    if(initialized) ovstage_shutdown();
    return code;
}
