for n=2:1:21
    for i=1:1:10
        nb = round(2000/n);
        a = poissrnd(nb, 1, n);
        filename = strcat('packet_', num2str(n), '_',num2str(i), '.csv');
        csvwrite(filename,a);
    end;
end;