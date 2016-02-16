function ac = allcomb(ws)
    ac = cell(1);
    if length(ws)==1
        ac = null;
    elseif length(ws)==2
        ac{1} = ws{1} + ws{2}*10;
    else
        s1 = ws(1);
        s2 = ws(2:end);
        s3 = allcomb(s2);
        s23 = [s2,s3];
        for ii = 1:length(s23)
            ac{ii} = s23{ii}*10 + s1{1};
        end
        ac = [ac,s3];
    end
    
end