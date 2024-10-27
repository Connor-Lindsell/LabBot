[Tri,Pts] = plyread('D:\Work\15\New folder\LinearUR10Link0.ply','tri')
trisurf(Tri,Pts(:,1),Pts(:,2),Pts(:,3)); 
colormap(gray); axis equal;
 