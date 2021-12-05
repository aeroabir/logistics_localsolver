"""
	Run localsolver for a particular problem
	problems are taken from Gehring and Homberger

"""
from convert2lsp import write2lsp
import glob
import os
import sys
import pandas as pd

# data_dir = 'C:\\localsolver_9_5\\examples\\cvrptw\\instances\\C2'
# data_dir = 'instances\\C2'
data_dir = 'instances\\200customers'  ## already in lsp format

filetype = '.TXT'  # '.xml'
datafiles = glob.glob(os.path.join(data_dir, '*'+filetype))
lspfile = os.path.join(data_dir, 'test.lsp')
outfile = os.path.join(data_dir, 'test.out')
csvfile = os.path.join(data_dir, 'All_200_localsolver.csv')
timelimit = {200: 180, 400: 240, 600: 300, 800: 360, 1000: 720}
runOnly = [200]
data = []
count = 0
for file in datafiles:
    problem = file.split('\\')[-1].split(filetype)[0]
    nbCustomers = int(problem.split('_')[1])*100
    if (len(runOnly) > 0 and nbCustomers in runOnly) or (len(runOnly)==0):
        if filetype in ('.TXT', '.txt'):
            lspfile = file
        else:
            write2lsp(file, lspfile, problem)
        outfile = os.path.join(data_dir, problem+'.out')
        lsp_command = f"localsolver cvrptw.lsp inFileName={lspfile} solFileName={outfile} lsTimeLimit={timelimit[nbCustomers]}"
        # lsp_command = "localsolver cvrptw.lsp inFileName="+lspfile+" solFileName="+outfile+" lsTimeLimit="+str(timelimit)
        print('>>>>>', problem, nbCustomers, outfile)
        print('>>>>>', lsp_command)
        os.system(lsp_command)
        with open(outfile, 'r') as fr:
            nbTrucksUsed, totalDistance = fr.readline().split()
        print(nbTrucksUsed, totalDistance)
        data.append((problem, nbTrucksUsed, totalDistance))
        count += 1
    # if count > 6:
    #     break

# os.remove(csvfile)
df = pd.DataFrame(data, columns =['Name', 'Trucks', 'Distance'])
df.to_csv(csvfile, index=False, header=True) 
print(f"Total {count} files written in {csvfile}")
