function visualizePose(object, objectEst)

    f2 = figure(randi(1000) + 500);
    f2.Position = [20 20 800 750];

    Pi = object;
    Po = objectEst;

    c1 = [15, 129, 186]./255;
    c2 = [1, 28, 14]./255;
    c3 = [150, 8, 79]./255;
    
    subplot(2,2,1);
    plot3(Pi(1,:), Pi(2,:), Pi(3,:), 'o', 'MarkerFaceColor',c1, 'MarkerEdgeColor',c1); hold on;
    axis on; grid on; axis equal;
    xlabel('X', 'Interpreter','latex'); ylabel('Y', 'Interpreter','latex'); zlabel('Z', 'Interpreter','latex');
    title('Groundtruth pose (3D)','Interpreter','latex'); hold off;
    
    subplot(2,2,2);
    plot3(Po(1,:), Po(2,:), Po(3,:), 'o', 'MarkerFaceColor',c2, 'MarkerEdgeColor',c2); hold on;
    axis on; grid on; axis equal;
    xlabel('X', 'Interpreter','latex'); ylabel('Y', 'Interpreter','latex'); zlabel('Z', 'Interpreter','latex');
    title('Estimated pose (3D)','Interpreter','latex'); hold off;
    
    subplot(2,2,3);
    plot3(Pi(1,:), Pi(2,:), Pi(3,:), 'o', 'MarkerFaceColor',c1, 'MarkerEdgeColor',c1); hold on;
    plot3(Po(1,:), Po(2,:), Po(3,:), 'o', 'MarkerFaceColor',c2, 'MarkerEdgeColor',c2); hold on;
    axis on; grid on; axis equal;
    xlabel('X', 'Interpreter','latex'); ylabel('Y', 'Interpreter','latex'); zlabel('Z', 'Interpreter','latex');
    legend({'Groundtruth', 'Estimated'}, 'Location','southoutside', 'Orientation','horizontal','Interpreter','latex');
    title('Groundtruth and output pose combined (3D)','Interpreter','latex'); hold off;
    
    subplot(2,2,4);
    B1 = boundary(Pi(1,:).', Pi(2,:).');
    B2 = boundary(Po(1,:).', Po(2,:).');
    plot(Pi(1,B1), Pi(2,B1), 'o', 'MarkerFaceColor',c1, 'MarkerEdgeColor',c1, 'MarkerSize',8); hold on;
    plot(Po(1,B2), Po(2,B2), 'o', 'MarkerFaceColor',c3, 'MarkerEdgeColor',c3, 'MarkerSize',5); hold on;
    axis off; grid off; axis equal;
    xlabel('X', 'Interpreter','latex'); ylabel('Y', 'Interpreter','latex');
    title('Input and output silhouettes (2D)','Interpreter','latex'); hold off;
    legend({'Input', 'Estimated'}, 'Location','southoutside', 'Orientation','horizontal','Interpreter','latex');
    
    hold off; drawnow();
    pause(0.1);
    
end