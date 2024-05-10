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
        
    end
end
