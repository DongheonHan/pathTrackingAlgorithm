% ORED LAB | DRV_MODEL_TEST | Version 1 2021-09-03
% Linear Dynamic Queue | Version 1 | 09/13/2021

% Queue Algorithms: (1) Linear Dynamic Queue, (2) Linear Static Queue, (3) Circular Queue
% Author: Dongheon Han

% Functions List
    % Constuctor Queue: The constructor determines the maximum capacity (length) of the data array
    % push: push enqueues the input data to the end of the queue and returns the object
    % pop: pop returns the object and the data at the beginning of the queue
    % getCount: Count the number of members in the array

% References
    % https://www.youtube.com/watch?v=jXMqVpAVyMY
    % https://allonlinejudgesolutions.blogspot.com/2019/01/an-algorithm-to-insert-new-element-in.html
    % https://www.programiz.com/dsa/types-of-queue

classdef QUEUE
    properties
        % Public Variables...
        
        % DATA ARRAY
        queueArray;     % Data Array
        
        % INDICES
        front = 1;      % Front Index
        rear = 1;       % Rear Index
        size;           % Maximum Array Size (e.g. 100)
        
        % COUNT ELEMENTS
        %elementCount;   % Count the number of the elements
    end
    
    methods
        %% Constructor
        function obj = QUEUE(size)
            obj.size = size;
        end
        
        %% Enqueue Data
        function obj = push(obj,inputVar)                           % Input Value: Enqueued Data
            if(obj.rear == obj.size + 1)                            % (Full || (Right Align && Not Full))
                if(1 == obj.front)                                  % The Array is FULL
                    obj.rear = obj.size + 1;
                    fprintf('Queue Full\n')
                else                                                % Right Align && Not Full
                    obj.queueArray(1) = inputVar;                   % Put Next Queue to array(1)
                    obj.rear = 2;
                end                                                 % End
            else
                if(obj.rear + 1 == obj.front)                       % When Rear meets Front
                    fprintf('Queue Full\n')
                else
                    obj.queueArray(obj.rear) = inputVar;
                    obj.rear = obj.rear + 1;
                end                                                 % End
            end
        end
        
        %% Dequeue Data
        function [obj, firstMember] = pop(obj)                      % Return Value: Dequeued Data
            if(obj.rear == obj.front)
                fprintf('Queue Empty\n')
                firstMember = NaN;
                return
            else
                if(obj.front == obj.size)                           % If Front data was placed very right
                    firstMember = obj.queueArray(obj.front);        % Front data saved to return value
                    obj.queueArray(obj.front) = NaN;
                    if(obj.front + 1 == obj.rear)                   % Single Element - start from index 1
                        obj.front = 1;
                        obj.rear = 1;
                    else
                        obj.front = 1;                              % Update Front: the next data became front
                    return                                          % End
                    end
                else
                    firstMember = obj.queueArray(obj.front);        % Front data saved to return value
                    obj.queueArray(obj.front) = NaN;
                    obj.front = obj.front + 1;                      % Update Front: the next data became front
                    return                                          % End
                end
            end
        end
        
        %% Count the number of members in the array
        function elemNum = getCount(obj)
            lengthArray = length(obj.queueArray);                   % total number of elements in the array (that includes NaNs (blancks))
            nanCount = sum(isnan(obj.queueArray));                  % total number of blanck elements (NaNs)
            elemNum = lengthArray - nanCount;                       % total number of members in the array: return this value as an output
        end
    end
end