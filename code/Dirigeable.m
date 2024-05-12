classdef Dirigeable

    properties
        Data
        Axes
    end
    
    methods
        function obj = Dirigeable(data, axes)
            obj.Data = data;
            obj.Axes = axes;
        end
        
        function W = weight(obj)
            norm_W = - obj.Data("m_struct") * obj.Data("gravity");
            z_m = obj.Axes("z_m");
            W = norm_W * z_m;
        end

        function B = Buoyancy(obj)
            norm_B = obj.Data("rho") * obj.Data("v_struct") * obj.Data("gravity");
            z_m = obj.Axes("z_m");
            B = norm_B * z_m;
        end

        function plot(obj)

            %% ---------------------------------- %%

            % avant
            vertices_1 = [
                2.2 0.5 -0.10; 
                1.8 0.5 -0.10;
                1.8 0.5 0.10;
                2.2 0.5 0.10;
                2.2 0.7 -0.10;
                1.8 0.7 -0.10;
                1.8 0.7 0.10;
                2.2 0.7 0.10];

            vertices_2 = [
                2.2 -0.5 -0.10; 
                1.8 -0.5 -0.10;
                1.8 -0.5 0.10;
                2.2 -0.5 0.10;
                2.2 -0.7 -0.10;
                1.8 -0.7 -0.10;
                1.8 -0.7 0.10;
                2.2 -0.7 0.10];
            
            % arrière
            vertices_3 = [
                -2.2 0.5 -0.10; 
                -1.8 0.5 -0.10;
                -1.8 0.5 0.10;
                -2.2 0.5 0.10;
                -2.2 0.7 -0.10;
                -1.8 0.7 -0.10;
                -1.8 0.7 0.10;
                -2.2 0.7 0.10];

            vertices_4 = [
                -2.2 -0.5 -0.10; 
                -1.8 -0.5 -0.10;
                -1.8 -0.5 0.10;
                -2.2 -0.5 0.10;
                -2.2 -0.7 -0.10;
                -1.8 -0.7 -0.10;
                -1.8 -0.7 0.10;
                -2.2 -0.7 0.10];

            %% ---------------------------------- %%

            % avant
            x_T_1 = 2.02;
            x_T_2 = 1.98;
            y_T_1 = 0.7;
            y_T_2 = 0.9;
            z_T = 0.02;

            vertices_T1 = [
                x_T_1 y_T_1 -z_T; 
                x_T_2 y_T_1 -z_T;
                x_T_2 y_T_1  z_T;
                x_T_1 y_T_1  z_T;
                x_T_1 y_T_2 -z_T;
                x_T_2 y_T_2 -z_T;
                x_T_2 y_T_2  z_T;
                x_T_1 y_T_2  z_T];

            % Utilisation des variables pour vertices_T2
           vertices_T2 = [
                x_T_1 -y_T_1 -z_T; 
                x_T_2 -y_T_1 -z_T;
                x_T_2 -y_T_1 z_T;
                x_T_1 -y_T_1 z_T;
                x_T_1 -y_T_2 -z_T;
                x_T_2 -y_T_2 -z_T;
                x_T_2 -y_T_2 z_T;
                x_T_1 -y_T_2 z_T];

            % arrière
            vertices_T3 = [
                -x_T_1 y_T_1 -z_T; 
                -x_T_2 y_T_1 -z_T;
                -x_T_2 y_T_1  z_T;
                -x_T_1 y_T_1  z_T;
                -x_T_1 y_T_2 -z_T;
                -x_T_2 y_T_2 -z_T;
                -x_T_2 y_T_2  z_T;
                -x_T_1 y_T_2  z_T];

            vertices_T4 = [
                -x_T_1 -y_T_1 -z_T; 
                -x_T_2 -y_T_1 -z_T;
                -x_T_2 -y_T_1 z_T;
                -x_T_1 -y_T_1 z_T;
                -x_T_1 -y_T_2 -z_T;
                -x_T_2 -y_T_2 -z_T;
                -x_T_2 -y_T_2 z_T;
                -x_T_1 -y_T_2 z_T];

            %% ---------------------------------- %%

            x_mot_1 = 2.1;
            x_mot_2 = 1.9;
            y_mot_1 = 0.9;
            y_mot_2 = 1.1;
            z_mot = 0.05;

            % avant
            vertices_mot_1 =[
                x_mot_1 y_mot_1 -z_mot; 
                x_mot_2 y_mot_1 -z_mot;
                x_mot_2 y_mot_1 z_mot;
                x_mot_1 y_mot_1 z_mot;
                x_mot_1 y_mot_2 -z_mot;
                x_mot_2 y_mot_2 -z_mot;
                x_mot_2 y_mot_2 z_mot;
                x_mot_1 y_mot_2 z_mot
                ];

            vertices_mot_2 =[
                x_mot_1 -y_mot_1 -z_mot; 
                x_mot_2 -y_mot_1 -z_mot;
                x_mot_2 -y_mot_1 z_mot;
                x_mot_1 -y_mot_1 z_mot;
                x_mot_1 -y_mot_2 -z_mot;
                x_mot_2 -y_mot_2 -z_mot;
                x_mot_2 -y_mot_2 z_mot;
                x_mot_1 -y_mot_2 z_mot
                ];

            % arrière
            vertices_mot_3 = [
                -x_mot_1 y_mot_1 -z_mot; 
                -x_mot_2 y_mot_1 -z_mot;
                -x_mot_2 y_mot_1 z_mot;
                -x_mot_1 y_mot_1 z_mot;
                -x_mot_1 y_mot_2 -z_mot;
                -x_mot_2 y_mot_2 -z_mot;
                -x_mot_2 y_mot_2 z_mot;
                -x_mot_1 y_mot_2 z_mot
                ];

            vertices_mot_4 = [
                -x_mot_1 -y_mot_1 -z_mot; 
                -x_mot_2 -y_mot_1 -z_mot;
                -x_mot_2 -y_mot_1 z_mot;
                -x_mot_1 -y_mot_1 z_mot;
                -x_mot_1 -y_mot_2 -z_mot;
                -x_mot_2 -y_mot_2 -z_mot;
                -x_mot_2 -y_mot_2 z_mot;
                -x_mot_1 -y_mot_2 z_mot
                ];

            %% ---------------------------------- %%
            x_nac = 0.25;
            y_nac = 0.15;
            z_nac_1 = -0.7;
            z_nac_2 = -0.9;

            vertices_nacelle = [
                -x_nac -y_nac z_nac_1;
                 x_nac -y_nac z_nac_1;
                 x_nac y_nac z_nac_1;
                -x_nac y_nac z_nac_1;
                -x_nac -y_nac z_nac_2;
                 x_nac -y_nac z_nac_2;
                 x_nac y_nac z_nac_2;
                -x_nac y_nac z_nac_2;
                ];
            
            x_grap_1 = 1.4;
            x_grap_2 = 1.6;
            y_grap = 0.20;
            z_grap_1 = -0.6;
            z_grap_2 = -0.8;

            vertices_grappin = [
                 x_grap_1 -y_grap z_grap_1;
                 x_grap_2 -y_grap z_grap_1;
                 x_grap_2 y_grap z_grap_1;
                 x_grap_1 y_grap z_grap_1;
                 x_grap_1 -y_grap z_grap_2;
                 x_grap_2 -y_grap z_grap_2;
                 x_grap_2 y_grap z_grap_2;
                 x_grap_1 y_grap z_grap_2;
                ];

            %% ---------------------------------- %%

            % Indices des faces du cube
            faces = [1 2 3 4; % Bas
                     5 6 7 8; % Haut
                     1 2 6 5; % Côtés
                     2 3 7 6;
                     3 4 8 7;
                     4 1 5 8];

            %% ---------------------------------- %%
            
            % Dessiner le cube
            figure;
            hold on;
            axis equal;
            grid on;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Visualisation des forces du dirigeable');
            
            % CAO structure moteur
            patch('Vertices', vertices_1, 'Faces', faces, 'FaceColor', 'r', 'FaceAlpha', 0.5);
            patch('Vertices', vertices_2, 'Faces', faces, 'FaceColor', 'r', 'FaceAlpha', 0.5);
            patch('Vertices', vertices_3, 'Faces', faces, 'FaceColor', 'r', 'FaceAlpha', 0.5);
            patch('Vertices', vertices_4, 'Faces', faces, 'FaceColor', 'r', 'FaceAlpha', 0.5);
            
            % CAO tube moteur
            patch('Vertices', vertices_T1, 'Faces', faces, 'FaceColor', 'b', 'FaceAlpha', 0.5);
            patch('Vertices', vertices_T2, 'Faces', faces, 'FaceColor', 'b', 'FaceAlpha', 0.5);
            patch('Vertices', vertices_T3, 'Faces', faces, 'FaceColor', 'b', 'FaceAlpha', 0.5);
            patch('Vertices', vertices_T4, 'Faces', faces, 'FaceColor', 'b', 'FaceAlpha', 0.5);

            % moteur
            patch('Vertices', vertices_mot_1, 'Faces', faces, 'FaceColor', 'g', 'FaceAlpha', 0.5);
            patch('Vertices', vertices_mot_2, 'Faces', faces, 'FaceColor', 'g', 'FaceAlpha', 0.5);
            patch('Vertices', vertices_mot_3, 'Faces', faces, 'FaceColor', 'g', 'FaceAlpha', 0.5);
            patch('Vertices', vertices_mot_4, 'Faces', faces, 'FaceColor', 'g', 'FaceAlpha', 0.5);

            % nacelle
            patch('Vertices', vertices_nacelle, 'Faces', faces, 'FaceColor', 'c', 'FaceAlpha', 0.5);

            % grappin
            patch('Vertices', vertices_grappin, 'Faces', faces, 'FaceColor', 'm', 'FaceAlpha', 0.5);
            
            %% ---------------------------------- %%
            
            % repère
            scale = 1;
            quiver3(0,0,0, scale, 0, 0, 'k', 'LineWidth',2)
            quiver3(0,0,0, 0, scale, 0, 'k', 'LineWidth',2)
            quiver3(0,0,0, 0, 0, scale, 'k', 'LineWidth',2)

            % thrust
            x_t = (x_mot_1 + x_mot_2)/2;
            y_t = (y_mot_1 + y_mot_2)/2;
            quiver3(x_t, y_t, z_mot, 0, 0, scale/2, 'Color', [1, 0.5, 0], 'LineWidth', 1)
            quiver3(-x_t, y_t, z_mot, 0, 0, scale/2, 'Color', [1, 0.5, 0], 'LineWidth', 1)
            quiver3(-x_t, -y_t, z_mot, 0, 0, scale/2, 'Color', [1, 0.5, 0], 'LineWidth', 1)
            quiver3(x_t, -y_t, z_mot, 0, 0, scale/2, 'Color', [1, 0.5, 0], 'LineWidth', 1)

            % weight
                % structure moteur
            navy = [0, 0, 0.5];
            x_p_sm = (x_T_1 + x_T_2)/2;
            y_p_sm = (y_T_1 + y_T_2)/2;
            quiver3(x_p_sm, y_p_sm, -z_T, 0, 0, -scale/2, 'Color', navy, 'LineWidth', 1.5)
            quiver3(-x_p_sm, y_p_sm, -z_T, 0, 0, -scale/2, 'Color', navy, 'LineWidth', 1.5)
            quiver3(-x_p_sm, -y_p_sm, -z_T, 0, 0, -scale/2, 'Color', navy, 'LineWidth', 1.5)
            quiver3(x_p_sm, -y_p_sm, -z_T, 0, 0, -scale/2, 'Color', navy, 'LineWidth', 1.5)

                % nacelle
            quiver3(0, 0, z_nac_2, 0, 0, -scale/2, 'Color', navy, 'LineWidth', 1.5)
                
                % grappin
            x_grap = (x_grap_1 + x_grap_2)/2;
            quiver3(x_grap, 0, z_grap_2, 0, 0, -scale/2, 'Color', navy, 'LineWidth', 1.5)
            
            % centre de gravité à vide
            G = [6*0.41, 0, 0];
            quiver3(3-G(1), 0, 0, 0, 0, -scale, 'Color', [0.5, 0, 0.5], 'LineWidth', 1.5)
            plot3(3-G(1), 0, 0, 'o', 'MarkerSize', 4, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.5, 0, 0.5]);
            text(3-G(1), 0, 0.05, 'G', 'FontSize', 12, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');


            %% ---------------------------------- %%

            % Définir les axes pour l'ellipsoïde
            a = 3;  % Rayon le long de l'axe x
            b = 0.75;  % Rayon le long de l'axe y
            c = 0.75;  % Rayon le long de l'axe z
            
            % Créer un maillage pour les angles theta et phi
            [theta, phi] = meshgrid(linspace(0, 2*pi, 50), linspace(0, pi, 50));
            
            % Calculer les coordonnées x, y, z de l'ellipsoïde
            x = a * sin(phi) .* cos(theta);
            y = b * sin(phi) .* sin(theta);
            z = c * cos(phi);
            
            % Dessiner l'ellipsoïde
            h = surf(x, y, z);
            h.FaceColor = 'blue';  % Ellipsoïde de couleur bleue
            h.EdgeColor = 'none';  % Pas de lignes de grille
            h.FaceAlpha = 0.25;  % Transparence à 50%
            
            % Ajustements visuels
            view(3);  % Vue 3D

            hold off;

        end
        
    end
end
