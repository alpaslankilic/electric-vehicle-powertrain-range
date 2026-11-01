function write_json(path,value)
%WRITE_JSON Save reproducible parameters and measured verification results.
file=fopen(path,'w');assert(file~=-1,'Cannot write result.');
cleanup=onCleanup(@() fclose(file));fprintf(file,'%s\n',jsonencode(value,'PrettyPrint',true));
end
