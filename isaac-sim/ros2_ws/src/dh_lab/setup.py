from setuptools import find_packages, setup

package_name = "dh_lab"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={"dh_lab.config": ["default_config.json"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=False,
    maintainer="mariana.reis",
    maintainer_email="cazevedo@ipn.pt",
    description=(
        "Teaching tool: parametric Craig-DH table vs. ground-truth forward "
        "kinematics on the simulated uFactory Lite6."
    ),
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dh_lab_grade = dh_lab.grading.headless_grade:main",
            "dh_lab_node = dh_lab.ros2_node.dh_lab_node:main",
        ],
    },
)
