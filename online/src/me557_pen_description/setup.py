from setuptools import find_packages, setup

package_name = 'me557_pen_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            'share/' + package_name + '/urdf',
            [
                'urdf/me557_pen.urdf.xacro',
                'urdf/Robot Assembly.SLDASM.urdf',
                'urdf/me557_pen.urdf',
            ],
        ),
        (
            'share/' + package_name + '/config',
            ['config/me557_pen_params.yaml', 'config/me557_pen_rviz.rviz'],
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/view_me557_pen.launch.py', 'launch/debug_urdf.launch.py'],
        ),
        (
            'share/' + package_name + '/meshes',
            [
                'meshes/base_link.STL',
                'meshes/Link1.STL',
                'meshes/Link2_L.STL',
                'meshes/Link3_R.STL',
                'meshes/Link4.STL',
                'meshes/Link5.STL',
                'meshes/Link6.STL',
                'meshes/Pen_tip.STL',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oteya',
    maintainer_email='oteya@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'write_on_board = me557_pen_description.write_on_board:main',
        ],
    },
)
