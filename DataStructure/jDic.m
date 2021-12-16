classdef jDic < jObj
% jDic is a wrapper class (Dictionary) used for the string controlled 
% container map for more convenient management of jObj based objects.

% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)        
        DicType=[]; %Dictionary Compatible Type (char)
        Dic=[]; %The Dictionary Container List (containers.Map)
        Content={}; %The List of Registered Content in Registering Order
    end
    
    methods(Access=public)
        function obj=jDic(inName,inType)
            % Constructor of jDic. [Original to jDic]
            % obj=jDic(inName,inType)
            % <inName>: NameTag (Char)
            % <inType>: Compatible Type (Char or EMPTY for unlimited type)
            obj@jObj(inName);
            if((ischar(inType)||isempty(inType)))
                obj.DicType=inType;
            else
                error(obj.msgStr('Error','InType need to be char!'));
            end
            
            obj.Dic=containers.Map();
        end
        
        function obj=reType(obj,inType)
            % Change Compatible Type. [Original to jDic]
            % obj=reType(obj,inType)
            % <inType>: Compatible Type (Char or EMPTY for unlimited type)
            if(~(ischar(inType)||isempty(inType)))
                error(obj.msgStr('Error','InType need to be char!'));
            else
                obj.DicType=inType;
            end
        end
        
        function output=merge(obj,varargin)
            % Merge Two Dictionaries of Compatible Types. [Original to jDic]
            % If any of them is unlimited dictionary, output will be a
            % jDic with unlimited type.
            % output=merge(obj,varargin)
            % <varargin>: [variable input argument] other merging jDic libraries
            if nargin==2
                obj2=varargin{1};
                outDicType=obj.mergeCheck_(obj2);
                output=jDic(strcat(obj.NameTag,'+',obj2.NameTag),outDicType);
                output.Dic=[obj2.Dic;obj.Dic];
                output.Content=output.list();
            else
                mid=obj;
                for ii=1:nargin-1
                    mid=mid.merge(mid,varargin{ii});
                end
                output=mid;
            end
        end
        
        function output=get(obj,varargin)
            % Get data from jDic. [Original to jDic]
            % If input is a cell, output of elements will be wrapped by a 
            % cell in same the sequence as the input element key sequence.
            % Otherwise, output will be a single data element.
            % output=get(obj,varargin)
            % <varargin>: [variable input argument] Keys of Data to
            % Retrieve.
            index=varargin(1:end);
            if(numel(index)==0)
                output=values(obj.Dic).';
                return;
            elseif(numel(index)==1)
                index=index{1};
            else
                error(obj.msgStr('Error','Use cell if you want to get multiple references!'));
            end
            
            if(isempty(index))
                output={};
                return;
            end
            
            if(ischar(index))
                output=obj.Dic(index);
            elseif(isa(index,'cell'))
                dexSize=numel(index);
                dexShape=size(index);
                output=cell(dexSize,1);
                for ii=1:dexSize
                    if(ischar(index{ii}))
                        output{ii}=obj.Dic(index{ii});
                    else
                        error(obj.msgStr('Error','Dictionary Key need to be char!'));
                    end
                end
                output=reshape(output,dexShape);
            else
                error(obj.msgStr('Error','Dictionary Key need to be char!'));
            end
        end
        
        function obj=reg(obj,varargin)
            % Register data to jDic. [Original to jDic]
            % Use column cell for multiple registrations.
            % obj=reg(obj,varargin)
            % <varargin (1)>: jObj object or index (keys) of objects.
            % <varargin (2)>: Objects to be registered (Not valid for jObj with no operator overloading).
            index=varargin(1:end-1);
            input=varargin{end};
            if(numel(index)==0)
                obj.jReg_(input);
                return;
            elseif(numel(index)==1)
                index=index{1};
            else
                error(obj.msgStr('Error','Use cell if you want to reg multiple or non-jObj data!'));
            end
            
            if(ischar(index))
                if(obj.typeCheck_(input))
                    obj.Dic(index)=input;
                    obj.Content=[obj.Content;index];
                else
                    error(obj.msgStr('Error','The dictionary content type does not match with input data!'));
                end
            elseif(isa(index,'cell')&&isa(input,'cell'))
                if(numel(index)==numel(input))
                    dexSize=numel(index);
                    for ii=1:dexSize
                        if(ischar(index{ii}))
                            if(obj.typeCheck_(input{ii}))
                                obj.Dic(index{ii})=input{ii};
                                obj.Content=[obj.Content;index{ii}];
                            end
                        else
                            error(obj.msgStr('Error','Dictionary Key need to be char!'));
                        end
                    end
                else
                    error(obj.msgStr('Error','Index and Assignment sizes don''t match!'));
                end
            else
                error(obj.msgStr('Error','Dictionary Key need to be char!'));
            end
        end
        
        function output=list(obj)
            % List Keys in Alphabetical Order. [Original to jDic]
            % output=list(obj)
            % obj=reg(obj,varargin)
            % [output]: Key List (cell)
            keyList=keys(obj.Dic);
            output=reshape(keyList,numel(keyList),1);
        end
    end
    
    methods(Access=protected)
        function output=typeCheck_(obj,inVar)
            % Check if object type matches the jDic type. [Original to jDic]
            % output=typeCheck_(obj,inVar)
            % <inVar>: the varible to be type checked
            if(isempty(obj.DicType))
                output=1;
                return;
            else
                output=0;
            end
            
            inClass=class(inVar);
            
            if(strcmp(obj.DicType,inClass))
                output=1;
            else
                error(obj.msgStr('Error','The dictionary content type does not match with input data!'));
            end
        end
        
        function output=mergeCheck_(obj,inDic)
            % Check if other jDic type matches the jDic type. [Original to jDic]
            % output=mergeCheck_(obj,inDic)
            % <inDic>: the jDic file to be merged
            output=0;
            if(isa(inDic,'jDic'))
                if(strcmp(obj.DicType,inDic.DicType))
                    output=obj.DicType;
                elseif(isempty(obj.DicType)||isempty(inDic.DicType))
                    output=[];
                else
                    error(obj.msgStr('Error','The dictionary types are not compatible!'));
                end
            else
                error(obj.msgStr('Error','The input is not dictionary based!'));
            end
        end
        
        function obj=jReg_(obj,inVar)
            % Register jObj. [Original to jDic]
            % obj=jReg_(obj,inVar)
            % <inVar>: the variable object to be registered
            if(isa(inVar,'jObj'))
                if(obj.typeCheck_(inVar))
                    obj.Dic(inVar.NameTag)=inVar;
                    obj.Content=[obj.Content;inVar.NameTag];
                end
            elseif(isa(inVar,'cell'))
                inSize=numel(inVar);
                for ii=1:inSize
                    if(isa(inVar{ii},'jObj'))
                        if(obj.typeCheck_(inVar{ii}))
                            obj.Dic(inVar{ii}.NameTag)=inVar{ii};
                            obj.Content=[obj.Content;inVar{ii}.NameTag];
                        end
                    else
                        error(obj.msgStr('Error','The input is not jbase!'));
                    end
                end
            else
                error(obj.msgStr('Error','The input is not jbase!'));
            end
        end
    end
end
