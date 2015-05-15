function [ t, P, V ] = read_state( fname )
%READ_STATE Summary of this function goes here
%   Detailed explanation goes here

%%
%fname = 'open-loop/feedback-left.dump';
fid = fopen(fname);
tline = 'a';
i = 0;
t = [];
P = [];
V = [];
while ischar(tline)
    tline = fgetl(fid);
    
    if ~ischar(tline)
        break
    end
    tline;
    if mod(i, 2) == 0
        p1 = '\[motor_state\] ([0-9]+) T([0-9]+):([0-9]+):([0-9]+\.[0-9]+).*';
        [tok, match, split] = regexp(tline, p1, 'tokens', 'match', 'split');
        tok = tok{:};
        t_c = str2double(tok{2}) * 3600.0 + str2double(tok{3}) * 60.0 + str2double(tok{4});
        t = [t; t_c];
    end
    if mod(i, 2) == 1
        p2 = '\(([-]*\d+\.\d+),([-]*\d+\.\d+)\)';
        [tok, match, split] = regexp(tline, p2, 'tokens', 'match', 'split');
        tok = [tok{:, :}];
        p = str2double(tok);
        p = p(1:2:end);
        v = str2double(tok);
        v = v(2:2:end);
        P = [P; p];
        V = [V; v];
        %        q = str2double(match)
 %       Q = [Q; q];
    end
   
    i = i + 1;
end
fclose(fid);

end

