

ref_left = dlmread('open-loop/ref-left.dump', '\n')
fb_left = importdata('open-loop/feedback-left.dump');

%%
p1 = '\[motor_state\] ([0-9]+) T([0-9]+):([0-9]+):([0-9]+\.[0-9]+).*';
p2 = '\(([-]*\d+\.\d+),([-]*\d+\.\d+)\)';%\s*\((\d+\.\d+),(\d+\.\d+)\)\s*\((\d+\.\d+),(\d+\.\d+)\)\s*\((\d+\.\d+),(\d+\.\d+)\)\s*\((\d+\.\d+),(\d+\.\d+)\)\s*\((\d+\.\d+),(\d+\.\d+)\)\s*\((\d+\.\d+),(\d+\.\d+)\)';

[tok, match, split] = regexp(fb_left{1}, p1, 'tokens', 'match', 'split');

[tok, match, split] = regexp(fb_left{2}, p2, 'tokens', 'match', 'split');
%%

ref_path   = 'open-loop/ref-left.dump';
track_path = 'open-loop/track-left.dump';
fb_path = 'open-loop/feedback-left.dump';

[t_ref, v_ref] = read_ref(ref_path);
[t_track, p_track, v_track] = read_state(track_path);
[t_fb, p_fb, v_fb] = read_state(fb_path);

%%
figure(1);
hold on;
grid on;
%plot(v_ref(:,4))
%plot(p_track(:,4))
%plot(v_fb(:,4))
plot(v_track(:,4) - v_fb(:,4))