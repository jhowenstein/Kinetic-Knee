function [ Z ] = shift_Z( Z0, ZREF )
    % Switch both values to positive (Clockwise convention)
    Z0 = abs(Z0);
    ZREF = abs(ZREF);
    
    if ZREF < 180
        if (Z0 >= ZREF && Z0 < (ZREF + 180))
            Z = Z0 - ZREF;
        elseif (Z0 < ZREF)
            Z = Z0 - ZREF;
        elseif Z0 >= (ZREF + 180)
            Z = (Z0 - 360) - ZREF;
        end
    elseif ZREF >= 180
        if ((Z0 < ZREF) && (Z0 >= (ZREF - 180)))
            Z = Z0 - ZREF;
        elseif Z0 >= ZREF
            Z = Z0 - ZREF;
        elseif Z0 < (ZREF - 180)
            Z = Z0 + (360 - ZREF);
        end
    else
        % Error!
        ;
    end
    
    % Flip Z rotation back to right hand convention
    Z = -Z;
end

