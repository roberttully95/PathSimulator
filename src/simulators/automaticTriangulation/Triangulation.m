classdef Triangulation
    %TRIANGULATION Triangulation enumeration class. As different triangulation methods are added,
    %they will be added to this enumeration class. This makes passing triangulations more readable.
    %Rather than passing '1' as an argument, you can pass 'Triangulation.Closest' which evaluates to
    %1.
    
    enumeration
        Closest
    end
end