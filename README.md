git clone --recurse-submodules git@github.com:zhangkunyi/quzhou-project.git

git submodule init
git submodule update



# remove submodule
git submodule deinit src/ego-planner-bspline
git rm src/ego-planner-bspline
rm -rf .git/modules/ego-planner-bspline
