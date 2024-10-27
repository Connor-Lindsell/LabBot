[Tri,Pts] = plyread('OurBotLink2.ply','tri')
trisurf(Tri,Pts(:,1),Pts(:,2),Pts(:,3)); 
colormap(gray); axis equal;
 