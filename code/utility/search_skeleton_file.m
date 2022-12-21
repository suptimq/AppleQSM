function [filename] = search_skeleton_file(tree_id, skel_folder, skel_filename_format)
    file_obj = dir(fullfile(skel_folder, [tree_id skel_filename_format]));
    filename = file_obj.name;
end
