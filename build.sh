#Windows
#pyinstaller --onefile main.py --collect-all roboticstoolbox --collect-all rtbdata --collect-all spatialgeometry --collect-all spatialmath --add-data "src/urdf/;src/urdf"
#Linux
pyinstaller --onefile main.py --collect-all roboticstoolbox --collect-all rtbdata --collect-all spatialgeometry --collect-all spatialmath --add-data "src/urdf/:src/urdf"