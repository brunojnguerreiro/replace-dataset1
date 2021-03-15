function msg = Struct2PointCloud(msgStruct)
%STRUCT2POINTCLOUD Copies fields from structure to a PointCloud2 object
    msg = rosmessage('sensor_msgs/PointCloud2');
    msg.Header.Seq = msgStruct.Header.Seq;
    msg.Header.Stamp.Sec = msgStruct.Header.Stamp.Sec;
    msg.Header.Stamp.Nsec = msgStruct.Header.Stamp.Nsec;
    msg.Header.FrameId = msgStruct.Header.FrameId;    
    fNames = fieldnames(msg);
    for k = 1:numel(fNames)
        if ~any(strcmp(fNames{k}, {'MessageType', 'Header','PreserveStructureOnRead','Fields'}))
            msg.(fNames{k}) = msgStruct.(fNames{k});
        end
    end
    for i = 1:length(msgStruct.Fields)
        tmp = rosmessage('sensor_msgs/PointField');
        fNames = fieldnames(tmp);
        for k = 1:numel(fNames)
            if ~any(strcmp(fNames{k}, {'MessageType','INT8','UINT8','INT16','UINT16','INT32','UINT32','FLOAT32','FLOAT64'}))
                tmp.(fNames{k}) = msgStruct.Fields(i).(fNames{k});
            end
        end
        msg.Fields(i) = tmp;
    end
end

