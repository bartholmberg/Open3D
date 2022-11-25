set BUILD_DIR="C:\repo\Open3D\build\lib\RelWithDebInfo\Python\cpu"
set VM_DIR="C:\venv2\o3d\Lib\site-packages\open3d\cpu"
copy /Y /V %BUILD_DIR%\*.dll %VM_DIR%\*.*
copy /Y /V %BUILD_DIR%\*.pyd %VM_DIR%\*.*
copy /Y /V %BUILD_DIR%\*.pdb %VM_DIR%\*.*
copy /Y /V C:\repo\apricus\fftw3\*.dll  %VM_DIR%\*