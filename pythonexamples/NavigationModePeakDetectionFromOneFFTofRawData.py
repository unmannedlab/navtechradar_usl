import matplotlib.pyplot as plt
from csv import reader

#########################################################################
bins_to_operate_on    =15     # The original Navtech c++ code coerces this between 5 & 15
start_bin             =10    # Allows for bins really close-in to be ignored
threshold             =85    # 85 is the value that one customer uses for target identification
range_gain            =1     # Radar specific value to scale the reported range-in-metres 
range_offset          =0     # Radar specific parameter to offset the reported range-in-metres
rangeresolutionmetres =0.044 # This value is rounded from the /real/ value to go in here!
max_peaks_per_azimuth =99    # maximum number of peaks 
filename="OneAzimuthFFT.CSV" # text file containing one line of comma separated integers
##########################################################################

awaiting_rise=False
data=[]

#=======================================================    
def square(number):
    return number ** 2
#=======================================================    
def cube(number):
    return number ** 3
#=======================================================    
def find_peak_bin (data,start_bin,end_bin,bins_to_operate_upon,threshold):
    global awaiting_rise
    if (start_bin > (end_bin - bins_to_operate_upon)):
        return end_bin 
    for peakBin in range (start_bin , end_bin - bins_to_operate_upon):    
        if (awaiting_rise and (data[peakBin] > data[peakBin + 1])):
            continue
        elif (awaiting_rise):
            awaiting_rise = False
        if ((data[peakBin] > threshold) and (data[peakBin + 1] < data[peakBin])) :
            awaiting_rise = True
            return peakBin
    return end_bin  
#=======================================================    
def peak_resolve(data,peak_bin,bins_to_operate_upon):
    x=[0]*bins_to_operate_on
    x2=[0]*bins_to_operate_on
    x3=[0]*bins_to_operate_on
    x4=[0]*bins_to_operate_on
    y=[0]*bins_to_operate_on
    xy=[0]*bins_to_operate_on
    x2y=[0]*bins_to_operate_on
    bins_to_offset = int((bins_to_operate_upon - 1) / 2 )   
    index = 0    
    startValue = peak_bin - bins_to_offset    
    for index in range (0, bins_to_operate_upon):
        x[index] = startValue+index
    #print (x)
    startBin = peak_bin - bins_to_offset 
    for index in range (0, bins_to_operate_upon):
        y[index] = data[startBin + index]    
    #print (y)
    Sx =  0.0
    Sx2 = 0.0
    Sx3 = 0.0
    Sx4 = 0.0    
    Sx=sum(x[0:bins_to_operate_upon])
    Sx2=sum(list(map(square,x[0:bins_to_operate_upon])))
    x3=list(map(cube,x[0:bins_to_operate_upon]))
    Sx3=sum(x3[0:bins_to_operate_upon])
    x2=list(map(square,x[0:bins_to_operate_upon]))
    x4=list(map(square,x2[0:bins_to_operate_upon]))
    Sx4=sum(x4[0:bins_to_operate_upon])
    Sy =   0.0
    Sxy =  0.0
    Sx2y = 0.0    
    Sy=sum(y[0:bins_to_operate_upon])
    xy=[X*Y for X,Y in zip(x[0:bins_to_operate_upon],y[0:bins_to_operate_upon])]
    Sxy=sum(xy[0:bins_to_operate_upon])
    x2y=[X*Y for X,Y in zip(x2[0:bins_to_operate_upon],y[0:bins_to_operate_upon])]
    Sx2y=sum(x2y[0:bins_to_operate_upon])
    A = [Sx2, Sx3, Sx4, Sx2y]
    B =[Sx, Sx2, Sx3, Sxy]  
    C = [ bins_to_operate_upon, Sx, Sx2, Sy ]   
    F = C[0] / A[0]    
    for index in range (0,4):
        C[index] = C[index] - (F * A[index])    
    F = B[0] / A[0]    
    for index in range (0,4):
        B[index] = B[index] - (F * A[index])    
    F = C[1] / B[1]    
    for index in range (0,4):        
        C[index] -= F * B[index]    
    b2 = C[3] / C[2]    
    b1 = (B[3] - B[2] * b2) / B[1]    
    return -b1 / (2 * b2) #+  startBin - (0 - bins_to_offset)    
#=======================================================    

with open(filename, 'r') as read_obj:
    FFTDataFromFile = reader(read_obj)
    data = list(map(int,(list(FFTDataFromFile))[0]))
print ("\n######### SETTINGS #########")
#print ("FFT values:      {}".format(data))
print ("Bins to operate: {}".format(bins_to_operate_on))
print ("Threshold:       {}".format(threshold))
print ("Range resolut'n: {}".format(rangeresolutionmetres))
print ("Range gain:      {}".format(range_gain))
print ("Range offset:    {}".format(range_offset))
print ("Max peaks:       {}".format(max_peaks_per_azimuth))
end_bin=len(data)
max_bins_to_operate_on = end_bin
minimum_range = bins_to_operate_on * rangeresolutionmetres
maximum_range = end_bin* rangeresolutionmetres
peaks_found=0
peak_bin=0
min_bin_to_operate_on=0
min_bin_to_operate_upon = start_bin
plt.figure("One Azimuth Analysis - Navigation Mode peak detection")
plt.title("Analysis of one Azimuth of FFT Radar data - 'Navigation Mode' peak detection\nPeakBin highlit in Orange, ResolvedPeak marked in green")
plt.plot(data, ".", markersize=5,color='b')
plt.hlines(threshold,start_bin,len(data),color='red')
while ((peak_bin != end_bin) and (peaks_found < max_peaks_per_azimuth)):
    peak_bin=find_peak_bin(data,min_bin_to_operate_upon,end_bin,bins_to_operate_on,threshold)
    min_bin_to_operate_upon = peak_bin + bins_to_operate_on
    print ("\n####### PEAK FOUND #########")
    print ("Peak bin found:  {}".format(peak_bin))
    if (peak_bin < end_bin):
        plt.plot(peak_bin,data[peak_bin],".-.",color='orange',markersize=12)
        resolvedBin= peak_resolve(data, peak_bin, bins_to_operate_on)
        print ("resolved bin at: {:.3f}".format(resolvedBin))
        resolvedRange = (resolvedBin *range_gain * rangeresolutionmetres) +range_offset
        print ("Resolved range:  {:.3f}m".format(resolvedRange))
        if ((resolvedRange < minimum_range) or (resolvedRange > maximum_range)):
            print ("implausible resolved range")
            continue
        peaks_found+=1
        plt.vlines(resolvedBin,min(data),max(data),color='green')
    else:
        print("Peak Bin >= end bin: ignore")
print ("\n############################")
plt.ylabel('Returned power', fontsize=12)
plt.xlabel('Reporting bin', fontsize=12)
plt.tight_layout()
plt.show()