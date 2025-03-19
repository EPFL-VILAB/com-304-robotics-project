# SCITAS Tutorial


- [SCITAS Tutorial](#scitas-tutorial)
- [What is SCITAS](#what-is-scitas)
- [Terminology](#terminology)
- [How to create an account](#how-to-create-an-account)
- [How to access to the cluster](#how-to-access-to-the-cluster)
  - [Login](#login)
  - [Volumes](#volumes)
    - [Home](#home)
    - [Scratch](#scratch)
  - [Upload data](#upload-data)
- [Running a job on the cluster](#running-a-job-on-the-cluster)
  - [Submitting a job](#submitting-a-job)
  - [GPU resources](#gpu-resources)
  - [Using Apptainer/Singularity to containerize your job](#using-apptainersingularity-to-containerize-your-job)
- [House keeping](#house-keeping)

---
# What is SCITAS
In the COM-304: Communications Project course, we are going to use the Scientific IT and Application Support (SCITAS) [[link](https://scitas-doc.epfl.ch/)] cluster for computation. It provides scientific computing resources and High-Performance Computing (HPC) services to everyone at EPFL. Currently, it contains four clusters: Helvetios, Izar GPU, Izar GPU 4x, and Jed [[link](https://scitas-doc.epfl.ch/supercomputers/overview/)]. We will mainly use [Izar](https://scitas-doc.epfl.ch/supercomputers/izar/) for our GPU computation.

# Terminology
When you see ***locally*** or ***on your own computer***, it means the command should be executed on your own computer, not the cluster.

When you see ***remotely*** or ***on the clusters***, it means the command should be executed on the clusters (after logging in), not your own computer.

We use `<username>` to denote your username on the clusters. Please replace it with your actual username when you execute the commands.

# How to create an account
You should be able to directly log into SCITAS with your GASPAR credentials when the COM-304 GPU allocation is confirmed. Your account will be associated with the COM-304 project to use reserved GPUs or acquire high priority in job queues.

# How to access to the cluster
To connect to the clusters, you have to be inside the EPFL network or [establish a VPN connection](https://www.epfl.ch/campus/services/en/it-services/network-services/remote-intranet-access/vpn-clients-available/) [[link](https://scitas-doc.epfl.ch/user-guide/using-clusters/connecting-to-the-clusters/)].

## Login
You can access the clusters by using `ssh` on your own computer. The command is:
```bash
ssh -X <username>@izar.epfl.ch
```

## Volumes
The volumes mentioned below are the folders existing on the clusters.

### Home
You have a 100 GB quota in `/home/<username>` for storing important files such as code, model, or anything that would be hard to reproduce. The files here are backed up every night and the storage is kept permanently. Note that we highly recommend maintaining a GitHub repository for your projects.

### Scratch
`/scratch/<username>` is used to store large datasets, checkpoints, etc. Files here are **NOT backed up** and the files **older than two weeks can get deleted**. Therefore, please only store files that you can afford to lose and reproduce easily here.

## Upload data
If need to upload data to the clusters or download data from the clusters [[link](https://scitas-doc.epfl.ch/user-guide/data-management/transferring-data/)]. On your computer, you can use `rsync` [[link](https://scitas-doc.epfl.ch/user-guide/data-management/transferring-data/#using-rsync)] (recommended) and `scp` [[link](https://scitas-doc.epfl.ch/user-guide/data-management/transferring-data/#using-scp)] if you prefer command line tools. If you prefer GUI applications, you can also use [WinSCP](https://winscp.net/eng/index.php) on Windows or [FileZilla](https://filezilla-project.org/) on MacOS and Linux locally.

# Running a job on the cluster
The SCITAS clusters use [SLURM](https://slurm.schedmd.com/documentation.html) to manage jobs submitted. It is a commonly used job scheduling system for HPC clusters. You can find almost all the information you want to know about SLURM on its [official documentation](https://slurm.schedmd.com/documentation.html) or by searching on ChatGPT/Google. 

## Submitting a job
You can submit a job by following the procedures below on the clusters.

1. Create a `xxx.run` file on the clusters. Here we call it `first_trial.run`. The content is
   ```bash
   #!/bin/bash
   #SBATCH --chdir /home/<username>
   #SBATCH --gres=gpu:1
   #SBATCH --ntasks 1
   #SBATCH --cpus-per-task 1
   #SBATCH --mem 4096
   #SBATCH --time 12:30:00 
   #SBATCH --output /home/<username>/logs/%j.out
   #SBATCH --account=com-304
   #SBATCH --qos=com-304

   echo STARTING AT `date`
   python --version
   echo FINISHED at `date`
   ```
   Let’s digest the commands a bit
   1. All the commands starting with `#SBATCH` specify the resources required for the job
      1. `#SBATCH --chdir /home/<username>` sets the working directory to be `/home/<username>`. All the commands are executed under this directory.
      2. `#SBATCH --gres=gpu:1` sets the number of GPUs required for the job to be `1`. Depending on the homework and the project track, you may need from 1 up to 8 GPUs. 
      3. `#SBATCH --ntasks 1` sets the number of parallel tasks to be `1`. In this course, you normally will only need 1 parallel task per job.
      4. `#SBATCH --cpus-per-task 1` sets the number of CPUs per task to be `1`. You can increase this number if your job is CPU-intensive.
      5. `#SBATCH --mem 4096` sets the amount of memory for the job to be 4096MB.
      6. `#SBATCH --time 12:30:00` sets the maximum living time of the job to be 12.5 hrs.
      7. `#SBATCH --output /home/<username>/logs/%j.out` sets the output from the job to be logged into `#SBATCH --output /home/<username>/logs/%j.out` files. `%j` denotes the job id.
      8. `#SBATCH --account=com-304` and `#SBATCH --qos=com-304` specify that we are using the GPUs reserved for the COM-304 course.
   2. The remaining commands are working scripts for training, etc. Here it simply prints the start and ending time of the script and the `python` version used.
      ```bash
      echo STARTING AT `date`
      python --version
      echo FINISHED at `date`
      ```
   For more information about the `xxx.run` file, you can check the `sbatch` page [[link](https://slurm.schedmd.com/sbatch.html)].
2. Submit the `first_trial.run` by `sbatch first_trial.run` on the clusters.
3. After submitting the job, you will see an immediate output `Submitted batch job 123456`. It means the job id is `123456`. Each job id is unique on the SLURM system.
4. You can cancel your job by running `scancel <job-id>`
5. To check the status of all your jobs, you can run `squeue -u <username>`.

## GPU resources
We have about 50 GPUs that the students in COM-304 have high priority to access to. These GPUs are shared with other users from EPFL but you have a high priority in the queue to access them. To use these GPUS, please specify the following in your `xxx.run` script.
```bash
#SBATCH --account=com-304
#SBATCH --qos=com-304
```

## Using Apptainer/Singularity to containerize your job
Please refer to the [tutorial](../RL_Habitat_Homework/README.md) for the RL notebook homework.

# House keeping

Please be considerate to the other students when using the clusters.

We do not restrict the number of GPUs each group can use. You can use all available resources, but other groups will not be able to use them, i.e., no preemption.
