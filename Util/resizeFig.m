function resizeFig(figHandle, newDims)
    % Resize a figure while preserving the top left position.
    %
    % Arguments
    % ---------
    % figHandle : figure handle
    %   Figure whose dimensions to modify.
    % newDims : (1, 2) int
    %   Desired dimensions (width, height) of the figure in point (1/72 inch).
    %
    % Authors
    % -------
    % Andrew Patterson
    % Tenavi Nakamura-Zimmerer
    %   NASA Langley Research Center
    %
    % Versions
    % --------
    %   2025-07-03: APP - Initial script created.
    %   2025-07-03: TNZ - Modified to preserve top left position instead of 
    %       center.
    arguments
        figHandle
        newDims (1, 2) int64 {mustBePositive}
    end

    % [left, bottom, width, height]
    oldPos = get(figHandle, 'Position');
    oldTop = oldPos(2) + oldPos(4);
    
    % Calculate new position to maintain upper left position
    newWidth = newDims(1);
    newHeight = newDims(2);

    newPos = [oldPos(1), oldTop - newHeight, newWidth, newHeight];
    
    % Apply new position
    set(figHandle, 'Position', newPos);
end
