%% BeiDou B1C Pilot Signal Secondary Code Generation
%---Generates the B1C Pilot secondary codes for tracking implementations

clear;
close all;
N = 3607;

BEIDOU_B1Cp_SECONDARY_TRUNCATION_POINT = ...
    {
    			1889, 1268, 1593, 1186, 1239, ...
			1930,  176, 1696,   26, 1344, ...
			1271, 1182, 1381, 1604, 1333, ... 
			1185,   31,  704, 1190, 1646, ...
			1385,  113,  860, 1656, 1921, ...
			1173, 1928,   57,  150, 1214, ... 
			1148, 1458, 1519, 1635, 1257, ... 
			1687, 1382, 1514,    1, 1583, ...
			1806, 1664, 1338, 1111, 1706, ...
			1543, 1813,  228, 2871, 2884, ...
			1823,   75,   11,   63, 1937, ...
			  22, 1768, 1526, 1402, 1445, ...
			1680, 1290, 1245
     };

BEIDOU_B1Cp_SECONDARY_PHASE_DIFFERENCE = ...
    {
    			 269, 1448, 1028, 1324,  822, ...
			   5,  155,  458,  310,  959, ...
			1238, 1180, 1288,  334,  885, ...
			1362,  181, 1648,  838,  313, ...
			 750,  225, 1477,  309,  108, ...
		        1457,  149,  322,  271,  576, ...
			1103,  450,  399,  241, 1045, ...
			 164,  513,  687,  422,  303, ...
			 324,  495,  725,  780,  367, ...
			 882,  631,   37,  647, 1043, ...
			  24,  120,  134,  136,  158, ...
			 214,  335,  340,  661,  889, ...
			 929, 1002, 1149
    };

bds_b1c_sec_pilot = strings(63,1);

%% Pilot secondary code generation
for i=1:63
    %--- Get weil code with w phase difference
    w = BEIDOU_B1Cp_SECONDARY_PHASE_DIFFERENCE{i};
    p = BEIDOU_B1Cp_SECONDARY_TRUNCATION_POINT{i};
    
    %--- Generate truncated weil code
    sec_pilot_seq = truncated_weil(N, w, p);
    
    %--- Secondary code string
    bds_b1c_sec_pilot(i) = sprintf('%d',sec_pilot_seq(1:1800));
    
    %--- Check values against ICD
    first_octal = fliplr(sec_pilot_seq(1:24));
    last_octal  = fliplr(sec_pilot_seq(77:100));
    fprintf('\nPRN: %d, First 24 Chips (Octal): %s First 24 Chips (Octal): %s \n', ...
        i, dec2base(bi2de(first_octal),8), dec2base(bi2de(last_octal),8));
end

%% Utility functions
function W = truncated_weil(N, w, p)

W=zeros(1,N);
count = 1;
for n=0: N-1
    k=mod(n+p-1, N);
    W(count) = xor(leg(k,N), leg(mod(k+w, N),N));

    count = count + 1;
end
end

function[W] = weil (p, w)

% WEIL Weil codes.
%    [w,W] = WEIL(P) generates a binary Weil code w and Weil code
%    sequence W.  P specifies the length of the code; it must be
%    a prime number.
%
%    WEIL(...,CODENUM) returns the NUM-th Weil code.  If missing,
%    default is CODENUM = 1.

W=zeros(1,p);
count = 1;
for k = 0:p-1
    
    W(count) = xor(leg(k,p), leg(mod(k+w, p),p));

    count = count + 1;
end

end

function L = leg(k,p)
% Compute the Legendre sequence

if (k == 0)
    L = 1;
else
    if (mod(k,p) == 0)
        L = NaN;   % p divides i (doesn't happen for primes)
    else
        squaremodp = 0;
        z = 1;
        while (z <= (p-1)/2)
            if (mod(z*z,p) == k)
                squaremodp = 1;
                break;
            end
            z = z + 1;
        end
        
        if (squaremodp)
            L = 0;   % i is a square(mod p)
        else
            L = 1;   % i is not a square(mod p)
        end
    end
end
end
