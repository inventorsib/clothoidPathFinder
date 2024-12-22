
x0 = [0, 0];
x = fmincon(@fun,x0,[],[],[],[],[-5, -5],[5, 5],@mycon);


function J = fun(x)
    J = x(1) + x(2);
end

function [c,ceq] = mycon(x)
    
    c = [];
    
    ceq = x(1)^2 + x(2)^2 - 1;

end

