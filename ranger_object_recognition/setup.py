from distutils.core import setup

setup(name='Ranger object recognition'
    , description='Object recognition for the Ranger robot'
    , author='Kian Dehmahdi'
    , packages=['ranger_object_recognition']
    , package_dir={'ranger_object_recognition': '.'}
    , entry_points={'console_scripts': ['ranger_find_item_in_scene = ranger_object_recognition.integration:find_item_in_scene']}
    , install_requires=[
        "numpy"
        "tensorflow"
        "opencv-contrib-python"
        "matplotlib"
    ]
    )