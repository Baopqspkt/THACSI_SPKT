% values = [12.7, 45.4, 98.9, 26.6, 53.1];
% [ave,stdev] = mystats(values);

function [avg, med] = mystats(x)
    n = length(x);
    avg = mymean(x,n);
    med = mymedian(x,n);


    function a = mymean(v,n)
        % MYMEAN Example of a local function.

        a = sum(v)/n;
    end


    function m = mymedian(v,n)
    % MYMEDIAN Another example of a local function.

    w = sort(v);
    if rem(n,2) == 1
        m = w((n + 1)/2);
    else
        m = (w(n/2) + w(n/2 + 1))/2;
    end
    end
end
