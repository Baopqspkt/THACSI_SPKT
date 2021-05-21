function [result_add,result_sub] = Test_Main
    x = 5;
    result_add = add(x);
    result_sub = sub(x);

end
 
function a = add(x)
        a = x + 5;
end  

function b = sub(x)
            b = x-1;
end
