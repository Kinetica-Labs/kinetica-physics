# Библиотека движка: C++17, только стандартная библиотека. Подключается через add_subdirectory.
# Заголовки — в include/, исходники — здесь, по тем же разделам.
cmake_minimum_required(VERSION 3.16)

add_library(kinetica-physics STATIC
  math/Matrix3x3.cpp
  math/Quaternion.cpp
  math/StepMemory.cpp
  math/UpdatedCholesky.cpp
  math/WorkerPool.cpp
  world/World.cpp
  world/VehicleConstraint.cpp
  world/IslandSleep.cpp
  world/AreaImpulse.cpp
  collision/ContactPair.cpp
  collision/ManifoldMemory.cpp
  collision/CandidateMemory.cpp
  collision/PairFinder.cpp
  collision/RayCast.cpp
  collision/broadphase/DynamicTree.cpp
  dynamics/event_driven/ContactSolver.cpp
  dynamics/event_driven/ContactRows.cpp
  dynamics/event_driven/PairFriction.cpp
  dynamics/event_driven/ContactAssembly.cpp
  dynamics/event_driven/CoupledSolve.cpp
  dynamics/event_driven/SlidingStopSearch.cpp
  dynamics/event_driven/WarmStart.cpp
  dynamics/event_driven/ShockPropagation.cpp
  dynamics/event_driven/ContactSystem.cpp
  dynamics/event_driven/ActiveSet.cpp
  dynamics/event_driven/SetFactor.cpp
  dynamics/event_driven/BodySpaceSolve.cpp
  dynamics/event_driven/ExactStep.cpp
  dynamics/event_driven/EventSearch.cpp
  dynamics/event_driven/OverlapSeparation.cpp
  body/Body.cpp
  body/MassFrame.cpp
  body/PrincipalAxes.cpp
  body/CompoundBodyBuilder.cpp
  dynamics/time_stepping/SoftStep.cpp
  dynamics/time_stepping/EquilibriumStep.cpp
  dynamics/time_stepping/SoftContact.cpp
  dynamics/time_stepping/IslandNormals.cpp
  dynamics/time_stepping/SupportNormals.cpp
  dynamics/time_stepping/ContactLevels.cpp
  dynamics/time_stepping/MissedContacts.cpp
  dynamics/time_stepping/PairReach.cpp
  dynamics/time_stepping/RollingFriction.cpp
  dynamics/time_stepping/CentreFriction.cpp
  dynamics/time_stepping/FrictionIslands.cpp
  dynamics/time_stepping/CoupledFriction.cpp
  dynamics/time_stepping/TrackedShapes.cpp
  dynamics/time_stepping/SoftJoint.cpp
  dynamics/time_stepping/DirectJoints.cpp
  joints/Joint.cpp
  joints/JointSet.cpp
  joints/SingleFreedomJoint.cpp
  joints/BallAndSocketJoint.cpp
  joints/HingeJoint.cpp
  joints/GearJoint.cpp
  joints/DistanceJoint.cpp
  joints/RackAndPinionJoint.cpp
  dynamics/WheelContact.cpp
  joints/SliderJoint.cpp
  joints/Path.cpp
  joints/PathJoint.cpp
  joints/CylindricalJoint.cpp
  joints/FixedJoint.cpp
  joints/CardanJoint.cpp
  joints/TetherJoint.cpp
  collision/shapes/ConvexShape.cpp
  collision/shapes/ConvexHull.cpp
  collision/shapes/Capsule.cpp
  collision/shapes/Cylinder.cpp
  collision/shapes/QuickHull.cpp
  collision/narrowphase/Gjk.cpp
  collision/narrowphase/Sat.cpp
  collision/narrowphase/NarrowPhaseAlgorithm.cpp
  collision/narrowphase/HullVsHullAlgorithm.cpp
  collision/narrowphase/HullVsRoundAlgorithm.cpp
  collision/narrowphase/RoundVsRoundAlgorithm.cpp
  collision/narrowphase/ContactPatch.cpp
  collision/narrowphase/ContactCandidate.cpp
  collision/shapes/ConvexDecomposition.cpp
  collision/shapes/Meshes.cpp
  debug/DebugGeometry.cpp)
target_include_directories(kinetica-physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_features(kinetica-physics PUBLIC cxx_std_17)
find_package(Threads REQUIRED)
target_link_libraries(kinetica-physics PUBLIC Threads::Threads) # пул потоков (math/WorkerPool.h)
# Слияние умножения со сложением (FMA) меняет биты от компилятора к компилятору — запрещено
# и движку, и всем, кто включает его заголовки.
target_compile_options(kinetica-physics PUBLIC -ffp-contract=off PRIVATE -Wall -Wextra -Wpedantic)
