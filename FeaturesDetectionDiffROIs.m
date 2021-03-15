function [Features, ROIs] = FeaturesDetectionDiffROIs(Image, No_Columns, No_Rows, No_Features, Thr)

    Image_Height = size(Image, 1);
    Image_Width = size(Image, 2);
    ROIs = CreateROIs(No_Columns, No_Rows, Image_Height, Image_Width);

    for i = (No_Columns+1):length(ROIs)    
        %temp_feat = detectSURFFeatures(rgb2gray(Image), 'MetricThreshold', 5000, 'NumOctaves', 1, 'NumScaleLevels', 6, 'ROI', ROIs{i});
        temp_feat = detectKAZEFeatures(rgb2gray(Image), 'Diffusion', 'region', 'Threshold', Thr, 'NumOctaves', 2, 'NumScaleLevels', 3, 'ROI', ROIs{i});    
        %temp_feat = detectBRISKFeatures(rgb2gray(Image), 'MinContrast', 0.75, 'MinQuality', 0.75, 'NumOctaves', 4, 'ROI', ROIs{i});    


        %temp_feat = detectMinEigenFeatures(rgb2gray(Image), 'ROI', ROIs{i});
        %temp_feat = detectKAZEFeatures(rgb2gray(Image), 'ROI', ROIs{i});
        %temp_feat = detectSURFFeatures(rgb2gray(Image), 'ROI', ROIs{i});
        %temp_feat = detectBRISKFeatures(rgb2gray(Image), 'ROI', ROIs{i});    

        %temp_feat = detectMSERFeatures(rgb2gray(Image), 'ROI', ROIs{i});

        % Bons Resultados SURF ('MetricThreshold', 5000, 'NumOctaves', 5, 'NumScaleLevels', 3)
        %temp_feat = detectSURFFeatures(rgb2gray(Image), 'MetricThreshold', 10000, 'NumOctaves', 6, 'NumScaleLevels', 3, 'ROI', ROIs{i});

        %temp_feat = detectHarrisFeatures(rgb2gray(Image));

        temp_feat = temp_feat.selectStrongest(1);

        Features_aux{i, :} = temp_feat;    
    end

    for i = 1:length(ROIs)
        if ~isempty(Features_aux{i, :})
            Metric(i, :) = Features_aux{i}.Metric  ;
        else
            Metric(i, :) = 0;
        end
    end

    [MaxMetric, MaxMetricIndex] = sort(Metric, 'descend');
    temp = nonzeros(MaxMetric);

    if isempty(temp)
        Features{1, :} = [];
    else
        for i = 1:min(No_Features, length(MaxMetric))
            if MaxMetric(i) ~= 0
                Features{i, :} = Features_aux{MaxMetricIndex(i), :};
            end    
        end
    end

end

function ROIs = CreateROIs(No_Columns, No_Rows, Image_Height, Image_Width)

    Widths{1} = [1 Image_Width/No_Columns];
    for i = 2:No_Columns
        Widths{i} = [(i-1)*Image_Width/No_Columns+1 (Image_Width/No_Columns)*i];
    end

    Heights{1} = [1 Image_Height/No_Rows];
    for i = 2:No_Rows
        Heights{i} = [(i-1)*Image_Height/No_Rows+1 (Image_Height/No_Rows)*i];
    end

    temp = 1;
    for i = 1:No_Rows
        for j = 1:No_Columns
            ROIs{temp} = [Widths{j}(1) Heights{i}(1) Image_Width/No_Columns Image_Width/No_Columns];
            temp = temp + 1;
        end
    end
  
end