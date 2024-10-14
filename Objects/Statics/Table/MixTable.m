scenario = robotScenario(UpdateRate=10);
addMesh(scenario,"Box",Position=[0.35 0 0.2],Size=[0.5 0.9 0.05],Color=[0.7 0.7 0.7],Collision="mesh")

visuals = "on";
collisions = "off";
show3D(scenario,Visuals=visuals,Collisions=collisions);
title("Scene with collision object-based static meshes")
view(81,19)
light
