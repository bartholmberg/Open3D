1) cd build
2) cmake -G "Visual Studio 17 2022"  -DCMAKE_TOOLCHAIN_FILE=C:/repo/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -A x64 ..
3) devenv
4) open open3d.sln in open3d/build
5) undo checkout of pipeline and python projects
6) rebuild 

6a) *may have to rebuild zlib with force link option







-------OLDBELOWLine-----------------------------
1) random nits, I think these are all fixed with vcpkg install

// Added a library and two include files 
// cpp\open3d\utility\ioapi.h cpp\open3d\utility\unzip.h 
// can get these files from your vcpkg install include dir 
// after --> vcpkg install zlib:x64-windows 

// also missing lib.  Here is how to get it
	1) vcpkg install zlib:x64-windows-static
	2) c:\repo\vcpkg\installed\x64-windows-static\lib\zlib.lib
	3) copy  zlib.lib to  👉 build\open3d\zlib\lib\zlibstatic.lib

The ExternalProjectTargets zlib has problems 😮.  
Better remove zlib subproject and use the vcpkg version until it is fixed

2) Make Conda3 use local open3d build

	copy output of open3d->Python->pybind vs project, by adding the following two lines
	to post build custom build in link section
	
	copy /Y $(OutDir)$(TargetName)$(TargetExt) $(OutDir)pybind$(TargetExt) 
	copy /Y $(OutDir)$(TargetName)$(TargetExt) C:\conda3\envs\o3d\Lib\site-packages\open3d\cpu\$(TargetName)$(TargetExt)
	

