#请修改为实际的vcpkg路径
toolchain_path='/workspace/vcpkg/scripts/buildsystems/vcpkg.cmake'

# 获取当前路径的父目录
current_path=$(pwd)
tmp1_dir=$(dirname "$current_path")
tmp2_dir=$(dirname "$tmp1_dir")
tmp3_dir=$(dirname "$tmp2_dir")
parent_dir=$(dirname "$tmp3_dir")

#安装aris
base_path_aris="$parent_dir/third_party/aris"
install_dir_aris="$parent_dir/install/aris"

python3 build_aris.py \
    --base-path "$base_path_aris" \
    --install-dir "$install_dir_aris" \
    --build-demo \
    --build-test \
    --build-all

#安装hpp-fcl
base_path_fcl="$parent_dir/third_party/hpp-fcl"
install_dir_fcl="$parent_dir/install/hpp-fcl"

python3 build_fcl.py \
    --base-path "$base_path_fcl" \
    --install-dir "$install_dir_fcl" \
    --toolchain-path "$toolchain_path" \
    --build-all

#安装UUID
base_path_uuid="$parent_dir/third_party/stduuid"
install_dir_uuid="$parent_dir/install/stduuid"

python3 build_uuid.py \
    --base-path "$base_path_uuid" \
    --install-dir "$install_dir_uuid" \
    --build-all

#安装sire
base_path_sire="$parent_dir"
install_dir_sire="$parent_dir/install/stduuid"

python3 build_aris.py \
    --bath-path "$parent_dir" \
    --aris-parh "$install_dir_aris" \
    --fcl-path "$install_dir_fcl" \
    --uuid-path "$install_dir_uuid" \
    --toolchain-path "$toolchain_path" \
    --build-demo \
    --build-all