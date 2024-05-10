classdef Moteur
    %MOTEUR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Data
        Data_m
        Axes_m
        Angle
    end
    
    methods
        function obj = Moteur(data, data_m, data_axes, angle)
            obj.Data = data;
            obj.Data_m = data_m;
            obj.Axes_m = data_axes;
            obj.Angle = angle;
        end

        function [a, b] = interp_rpm_v(obj)

            coefficients = polyfit(obj.Data_m("voltage"), obj.Data_m("estimated_RPM"), 1);
            a = coefficients(1);
            b = coefficients(2);

            x = linspace(10, 40, length(obj.Data_m("voltage")));
            y = a*x + b;
            
            legendText = sprintf('y = %.2fx + %.2f', a, b);

            figure
            plot(obj.Data_m("voltage"), obj.Data_m("estimated_RPM"), "bo--", "LineWidth", 1)
            hold on;
            plot(x, y, "r--", "LineWidth", 2)
            grid on;
            xlabel("Tension [V]")
            ylabel("RPM")
            title("Interpolation Volt / RPM ")
            legend("Données RPM(V)", legendText, 'Location', 'southeast')
            hold off;

        end

        function mean_Ct = average_C_t(obj)

            Ct = zeros(1, length(obj.Data_m("thrust")));
            RPM = obj.Data_m("estimated_RPM");
            thrust = obj.Data_m("thrust");

            for i = 1:length(obj.Data_m("thrust"))
                n = RPM(i)/60;
                Ct(i) = thrust(i)/ (obj.Data("rho") * n^2 * obj.Data("diameter")^4);
            end

            mean_Ct = mean(Ct);

        end

        function interp_thrust(obj, a, b)

            rho = obj.Data("rho");
            diameter = obj.Data("diameter");

            u = linspace(0, 41, 10);
            mean_ct = obj.average_C_t();

            RPM = linspace(0, 30000);
            T = mean_ct * rho .* (RPM./60).^2 * diameter^4;
            
            RPM_interp_1 = a .* u;
            T_1 = mean_ct * rho .* (RPM_interp_1./60).^2 * diameter^4;
            RPM_interp_2 = a .* u + b;
            T_2 = mean_ct * rho .* (RPM_interp_2./60).^2 * diameter^4;

            figure
            hold on;
            plot(RPM, T, "b", 'LineWidth', 2)
            plot(RPM_interp_1, T_1, 'k*', "LineWidth", 1)
            plot(RPM_interp_2, T_2, 'ro', "LineWidth", 1)
            xlabel("RPM")
            ylabel("Thrust [N]")
            title("Poussée d'un moteur en fonction des RPMs")
            legend("RPM", "RPM(u) = a*u","RPM(u) = a*u + b", 'Location', 'southeast')
            grid on;
            hold off;
        end

        function R_y = rotation_G_M(obj)
            alpha = obj.Angle;
            R_y = [[cos(alpha), 0, sin(alpha)];
                   [0, 1, 0];
                   [-sin(alpha), 0, cos(alpha)]];
        end

        function [F_prop, M_prop] = Force_Moment(obj, value_interp)
            R_y = obj.rotation_G_M();
            z_r = R_y * obj.Axes_m("z_m");

            puissance = value_interp("puissance");
            a = value_interp('a');
            b = value_interp('b');
            u = value_interp('voltage');
        
            if puissance <= 50
                RPM = a * u;
            else
                RPM = a * u + b;
            end
        
            norm_T = value_interp('Ct') * obj.Data('rho') * (RPM / 60)^2 * obj.Data('diameter')^4;

            F_prop = norm_T * z_r;
            M_prop = [0; 0; 0];
        end

    end
end

