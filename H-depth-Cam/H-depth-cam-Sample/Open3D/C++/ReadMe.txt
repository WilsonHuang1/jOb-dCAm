前置条件：
	[Visual Studio (2017及以上版本)](https://visualstudio.microsoft.com/vs/community/)
	[CMake (3.19及以上版本)](https://cmake.org/download/)
	[Open3D (0.15.1版本)](https://github.com/isl-org/Open3D)

设置Open3d路径
	1.在CmakeList.txt中增加open3d路径（eg:list(APPEND CMAKE_PREFIX_PATH "D:/Open3d/open3d-devel-windows-amd64-0.15.1")）
	
CMake运行：
	1. 运行CMake并且设置源码和生成路径
	2. 点击“配置”按钮，根据需求选择生成器和平台，再点击“结束”按钮
	3. 当配置结束后，点击“生成”按钮，生成工程文件

注意：
	运行SimpleView_ImageRead前，需先执行SimpleView_GetPointCloudImage和SimpleView_SaveImage，将保存的图像拷贝到程序运行目录下

Prerequisites:
    [Visual Studio (version 2017 or above)](https://visualstudio.microsoft.com/vs/community/)
	[CMake (version 3.19 or above)](https://cmake.org/download/)
	[Open3D (version 0.15.1 or above)](https://github.com/isl-org/Open3D)
	
Set Open3d library
	1.Set open3d library in CmakeList.txt（eg:list(APPEND CMAKE_PREFIX_PATH "D:/Open3d/open3d-devel-windows-amd64-0.15.1")）

CMake:
	1. Run CMake, set the source and build paths
	2. Click the "Configure" button. In the pop-up window, set the generator and platform according to the actual situation, and then click the "Finish" button.
	3. Click the "Generate" button after finish configure. When the log displays "Generating done", the project file is generated
	
Note:
	Before running the SimpleView_ImageRead, please execute the SimpleView_GetPointCloudImage and SimpleView_SaveImage and copy the saved images in the program running directory
