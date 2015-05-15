function [ t, Q ] = read_ref( fname )
%READ_REF Summary of this function goes here
%   Detailed explanation goes here

%%
% fname = 'open-loop/ref-left.dump'
fid = fopen(fname);
tline = 'a';
i = 0;
t = [];
Q = [];
while ischar(tline)
    tline = fgetl(fid);
    
    if ~ischar(tline)
        break
    end
    tline;
    if mod(i, 3) == 0
        p1 = '\[motor_ref\] ([0-9]+) T([0-9]+):([0-9]+):([0-9]+\.[0-9]+).*';
        [tok, match, split] = regexp(tline, p1, 'tokens', 'match', 'split');
        tok = tok{:};
        t_c = str2double(tok{2}) * 3600.0 + str2double(tok{3}) * 60.0 + str2double(tok{4});
        t = [t; t_c];
    end
    if mod(i, 3) == 2
        p2 = '\d+\.\d+';
        [match, split] = regexp(tline, p2, 'match', 'split');
        q = str2double(match);
        Q = [Q; q];
    end
   
    i = i + 1;
end
fclose(fid);

% plot(t, Q)


end

