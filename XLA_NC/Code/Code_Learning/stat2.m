% values = [12.7, 45.4, 98.9, 26.6, 53.1];
% [ave,stdev] = stat2(values);

function [m,s] = stat2(x)
    n = length(x);
    m = avg(x,n);
    s = sqrt(sum((x-m).^2/n));

    function m = avg(x,n)
        m = sum(x)/n;
    end
end
