function filteredData = medianFilter(data, windowSize)
    n = length(data);
    filteredData = zeros(size(data));
    halfWindow = floor(windowSize / 2);
    
    for i = 1:n
        windowStart = max(1, i - halfWindow);
        windowEnd = min(n, i + halfWindow);
        filteredData(i) = median(data(windowStart:windowEnd));
    end
end
