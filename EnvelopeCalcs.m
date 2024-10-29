  %% Limited joint Point coloud
        function EnvelopeCalcs()
            robot = CustomBot;
            %robot.model.base = robot.model.base.T * transl(0, 0, 0) % transl(0, 0.25, 0.5)% Y,Z,X
            robot.model.plotopt = {'nojoints', 'noname', 'noshadow', 'nowrist'};
            hold on
            workspace = [-2 2 -2 2 -2 2];
            scale = 1;
            stepRads = deg2rad(40);
            gndHeight = 0;
            stepLin = 0.1;
            qlim = robot.model.qlim
            pointCloudeSize = prod(floor((qlim(2:5,2)-qlim(2:5,1))/stepRads)) * prod(floor(((qlim(1,2)-qlim(1,1))/stepLin) + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            tic
            

for q1 = qlim(1,1):stepLin:qlim(1,2)
    for q2 = qlim(2,1):stepRads:qlim(2,2)
        for q3 = qlim(3,1):stepRads:qlim(3,2)
            for q4 = qlim(4,1):stepRads:qlim(4,2)
            % q4 = 0;
                for q5 = qlim(5,1):stepRads:qlim(5,2)
                    % q5 = 0;
                        %for q6 = qlim(6,1):stepRads:qlim(6,2)
                            q6 = 0;
                                q7=0;
                     
                        q = [q1,q2,q3,q4,q5,q6,q7];
                        tr = robot.model.fkine(q).T;  
                        %if tr(3,4)' > -1
                        pointCloud(counter,:) = tr(1:3,4)';
                        counter = counter + 1; 
                        if mod(counter/pointCloudeSize * 100,1) == 0
                            display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                        end
                end 
             end
        end
    end
end
% plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');


xPoints = [pointCloud(:,1)];
yPoints = [pointCloud(:,2)];
zPoints = [pointCloud(:,3)];



upperPoints = zPoints >= gndHeight; % pointCloud(zPoints<0) = []; %% remove all points below ground

xUpperPoints = xPoints(upperPoints);
yUpperPoints = yPoints(upperPoints);
zUpperPoints = zPoints(upperPoints);

pCmat = [xUpperPoints, yUpperPoints, zUpperPoints];


[kxz,xzAv] = convhull([pCmat(:,1),pCmat(:,3)], 'Simplify',true);
[kyz,yzAv] = convhull([pCmat(:,2),pCmat(:,3)], 'Simplify',true);
[kxy,xyAv] = convhull([pCmat(:,1),pCmat(:,2)], 'Simplify',true);

[k,av] = convhull(pCmat, 'Simplify',true);


%figure(1)
trisurf(k, xUpperPoints, yUpperPoints, zUpperPoints, 'FaceColor', 'cyan','EdgeColor', 'none' ,'FaceAlpha', 0.1)


% figure(2)
 %plot(pCmat(kxy,1),pCmat(kxy,2)); % XY Envelope
 %hold on
  %plot(1.2.*pCmat(kxy,1),1.2.*pCmat(kxy,2)); % XY Envelope
% axis equal
% figure(3)
% plot(pCmat(kxz,1),pCmat(kxz,3)); % XZ Envelope
% axis equal
% figure(4)
% plot(pCmat(kyz,2),pCmat(kyz,3)); % YZ Envelope
%axis equal

%plot3(pCmat(:,1),pCmat(:,2),pCmat(:,3),'r.');
av
 message = sprintf(num2str(av),2,'significant');
 text_h = text(10, 50, message, 'FontSize', 10, 'Color', [.6 .2 .6]);

axis equal
% hold on
% robot.model.plot3d(q,'workspace',workspace,'scale',scale);

        end