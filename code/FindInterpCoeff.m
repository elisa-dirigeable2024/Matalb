function coefficients = FindInterpCoeff(PropName, gravity, folder, coefficients)

    for i = 1:length(PropName)
        prop_name = PropName(i);
        prop_data = folder.(prop_name);
    
        Thrust_g_moy = prop_data{:, "Poussée moy (g)"} * gravity * 10^(-3);
        Voltage_moy = prop_data{:, "Voltage moy (V)"};
        Intensity_moy = prop_data{:, "Courant moy (A)"};
        
        % Calcul des coefficients pour Thrust vs Voltage
        [TV_a, TV_b, TV_c] = Coeffs(Voltage_moy, Thrust_g_moy);
        V = linspace(min(Voltage_moy), max(Voltage_moy), 1000);
        inter_pol_TV = TV_a .* V.^2 + TV_b .* V + TV_c;
        
        % Calcul des coefficients pour Thrust vs Current
        [TI_a, TI_b, TI_c] = Coeffs(Intensity_moy, Thrust_g_moy);
        I = linspace(min(Intensity_moy), max(Intensity_moy), 1000);
        inter_pol_TI = TI_a .* I.^2 + TI_b .* I + TI_c;
    
        % Calcul des coefficients pour Voltage vs Current
        [VI_a, VI_b, VI_c] = Coeffs(Intensity_moy, Voltage_moy);
        inter_pol_VI = VI_a .* I.^2 + VI_b .* I + VI_c;
        
        % Préparer le texte pour la légende
        legendText_TV = sprintf('T = %.3fV^2 + %.3fV + %.3f', TV_a, TV_b, TV_c);
        legendText_TI = sprintf('T = %.3fI^2 + %.3fI + %.3f', TI_a, TI_b, TI_c);
        legendText_VI = sprintf('V = %.3fI^2 + %.3fI + %.3f', VI_a, VI_b, VI_c);
        
        % Créer des sous-graphes pour Thrust vs Voltage
        subplot(3, 3, 3*(i-1)+1);
        hold on;
        plot(Voltage_moy, Thrust_g_moy, 'k*', 'LineWidth', 1);
        plot(V, inter_pol_TV, 'b--', 'LineWidth', 1.2);
        xlabel("Voltage [V]");
        ylabel("Poussée [N]");
        legend("Points", legendText_TV, 'Location', 'southeast');
        title(sprintf("Prop : %s - Thrust vs Voltage", prop_name));
        grid on;
        hold off;
        
        % Créer des sous-graphes pour Thrust vs Current
        subplot(3, 3, 3*(i-1)+2);
        hold on;
        plot(Intensity_moy, Thrust_g_moy, 'k*', 'LineWidth', 1);
        plot(I, inter_pol_TI, 'b--', 'LineWidth', 1.2);
        xlabel("Current [A]");
        ylabel("Poussée [N]");
        legend("Points", legendText_TI, 'Location', 'southeast');
        title(sprintf("Prop : %s - Thrust vs Current", prop_name));
        grid on;
        hold off;
    
        % Créer des sous-graphes pour Voltage vs Current
        subplot(3, 3, 3*(i-1)+3);
        hold on;
        plot(Intensity_moy, Voltage_moy, 'k*', 'LineWidth', 1);
        plot(I, inter_pol_VI, 'b--', 'LineWidth', 1.2);
        xlabel("Current [A]");
        ylabel("Voltage [V]");
        legend("Points", legendText_VI, 'Location', 'southeast');
        title(sprintf("Prop : %s - Voltage vs Current", prop_name));
        grid on;
        hold off;

        coefficients.(prop_name).Thrust_vs_Voltage = [TV_a, TV_b, TV_c, Voltage_moy(1), Voltage_moy(end)];
        coefficients.(prop_name).Thrust_vs_Current = [TI_a, TI_b, TI_c, Intensity_moy(1), Intensity_moy(end)];
        coefficients.(prop_name).Voltage_vs_Current = [VI_a, VI_b, VI_c, Intensity_moy(1), Intensity_moy(end)];
    end
end


function [a, b, c] = Coeffs(x,y)
    coefficients = polyfit(x, y, 2);
    a = coefficients(1);
    b = coefficients(2);
    c = coefficients(3);
end


