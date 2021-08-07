function bool = isToolboxAvailable(toolbox_name)
% check if MATLAB toolbox with given name is available
bool = any(any(contains(struct2cell(ver), toolbox_name)));