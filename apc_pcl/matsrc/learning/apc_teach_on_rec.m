function [sets] = apc_teach_on_rec(image_set)
% Teach on data recorded using the pcl.launch train:=true utility

sets = {}

for k = 1:length(image_set)
    item = image_set{k}
    sets = apc_manual_train_histo(item.image, item.name, 10, sets);
    disp(['Recorded ', num2str(length(sets)), 'entries'])
end