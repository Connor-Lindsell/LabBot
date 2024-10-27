function plyPatch = createPLYPatch(plyFile)
   [ faces, vertices, plyData ] = plyread(plyFile,'tri');

    % Create patch object
    plyPatch = patch('Vertices', vertices, 'Faces', faces, ...
                     'FaceColor', 'cyan', 'EdgeColor', 'none', ...
                     'FaceAlpha', 0.7);
end