classdef MotorSimulation
    
    properties
        MotorName;
        Coeff;
        AngleRot;
        Axes;
        InterpCoeff;
        Voltage;
    end 
    
    methods
        function obj = MotorSimulation(motorname, coeff, angle, data_axes, voltage_value)
            obj.MotorName = motorname;
            obj.Coeff = coeff;
            obj.AngleRot = angle;
            obj.Axes = data_axes;
            obj.Voltage = voltage_value;

            obj.InterpCoeff = obj.Coeff.(obj.MotorName).Thrust_vs_Voltage;
        end

        function plot_data(obj)
            a = obj.InterpCoeff(1);
            b = obj.InterpCoeff(2);
            c = obj.InterpCoeff(3);

            voltage_vector = linspace(obj.InterpCoeff(end-1), obj.InterpCoeff(end), 1000);
            Thrust_interp = a .* voltage_vector .^2 + b .* voltage_vector + c;
            
            legend_text = sprintf('T = %.3fV^2 + %.3fV + %.3f', a, b, c);
            
            figure
            plot(voltage_vector, Thrust_interp, "b--", "LineWidth", 2)
            xlabel("Voltage [V]")
            ylabel("Thrust [N]")
            legend(legend_text)
            grid on;
            title("Prop : " + obj.MotorName + " Thrust-Voltage")
            
        end

        function Rotation_matrix = Rotation(obj)
            c = cos(obj.AngleRot);
            s = sin(obj.AngleRot);

            Rotation_matrix = [
                [c, 0, s]
                [0, 1, 0]
                [-s, 0, c]
                ];
        end

        function F_vect = ForceExpression(obj)
            a = obj.InterpCoeff(1);
            b = obj.InterpCoeff(2);
            c = obj.InterpCoeff(3);
            T_norm = a .* obj.Voltage .^2 + b .* obj.Voltage + c;
            
            % Moteur en position initial selon z_r 
            % z_r = Rotation * z_m
            
            F_vect = T_norm * obj.Rotation() * obj.Axes("z_m");
        end

    end
end

