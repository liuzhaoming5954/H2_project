from setuptools import find_packages, setup

setup(
    name="h2_supervisor",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/h2_supervisor"]),
        ("share/h2_supervisor", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="H2 project team",
    maintainer_email="7611578+liuzhaoming5954@users.noreply.github.com",
    description="Simulation health checks and expiring motion permission leases",
    license="Proprietary",
    entry_points={"console_scripts": [
        "supervisor = h2_supervisor.supervisor_node:main",
    ]},
)
