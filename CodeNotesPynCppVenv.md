
rem 👉🏽  Got your 💘 fav python venv and 
rem     want to use in in C++ 
rem 👉🏽 
rem 👉🏽  Embedded python with venv
rem 👉🏽  Set PYTHONHOME=C:\Python310   or wherever 🤷🏻‍ your 🐍 python(&version) lives


rem 👉🏽 my 🐍 home
set PYTHONHOME=C:\Python310
rem 👉🏽 💘 fav python venv 
set MYVENV=C:\venv2\o3d 

ren %PYTHONHOME%\Lib\site-packages  site-packages(old)
c:\repo\apricus\tool\junction -s -q -nobanner %PYTHONHOME%\Lib\site-packages  %MYVENV%\Lib\site-packages

rem and retur to its former (un)glory
c:\repo\apricus\tool\junction -d %PYTHONHOME%\Lib\site-packages
ren %PYTHONHOME%\Lib\site-packages(old)  site-packages

rem and if you want to delete junction
rem ..\apricus\tool\junction -d %junctionDir% 