function [filename] = search_skeleton_file(tree_id, skel_folder, skel_filename_format)
    file_obj = dir(fullfile(skel_folder, [tree_id skel_filename_format]));
    if ~isempty(file_obj)
        filename = file_obj.name;
    else
        filename = nan;
    end
end
