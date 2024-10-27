function attachPLYToLink(plyFile, transformMatrix, ax)
    ptCloud = pcread(plyFile); % Load the PLY file
    vertices = ptCloud.Location;
    faces = convhull(vertices); % Generate faces from the point cloud

    % Create the patch object (3D mesh)
    patch('Vertices', vertices, 'Faces', faces, ...
          'FaceColor', 'cyan', 'EdgeColor', 'none', ...
          'FaceAlpha', 0.5, 'Parent', hgtransform('Matrix', transformMatrix, 'Parent', ax));
end