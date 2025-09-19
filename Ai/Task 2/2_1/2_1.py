import numpy as np
import pandas as pd
import csv
from sklearn.preprocessing import StandardScaler
import helpers
import os
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator


#------------------
#Loading file
#------------------
script_dir = os.path.dirname(os.path.abspath(__file__))

# Build paths to CSVs
results_path = os.path.join(script_dir, "results.csv")
races_path   = os.path.join(script_dir, "races.csv")
status_path  = os.path.join(script_dir, "status.csv")

#skipping bad lines and quotes 
df_results = pd.read_csv(results_path, on_bad_lines='skip').replace(r'"', '', regex=True)
df_races   = pd.read_csv(races_path,   on_bad_lines='skip').replace(r'"', '', regex=True)
df_status  = pd.read_csv(status_path,  on_bad_lines='skip').replace(r'"', '', regex=True)

#merging files 
df = df_results.merge(df_races, on="raceId", how="left",suffixes=["","_race"])

df = df.merge(df_status, on="statusId", how="left",suffixes=["","_status"])
#------------------
#Checking data
#------------------

#df.info()                           #to get each column and its type
# print("\n")

# print(df.describe())                #specs and quantiles
print("\n")

#print(df)                    #an example of the start of the set
#print(df['raceId'].value_counts())  #check for bias


#------------------
#Cleaning
#------------------
df.columns = df.columns.str.strip()                                   #remove spaces from columns names

#drop columns with more than 80% empty values before anything
bad_col_percent = df.apply(helpers.empty_percentge)
bad_cols_to_drop = bad_col_percent[bad_col_percent > 0.75].index     #increase threshold to be more strict
df.drop(columns=bad_cols_to_drop, inplace=True)
#Type_conversion  
numerical_cols = ['resultId','raceId','driverId','constructorId','grid'
               ,'positionOrder','points','laps','statusId','year','round','circuitId']
for col in numerical_cols :
    df[col] = df[col].apply(helpers.try_word_to_num)
    df[col] = pd.to_numeric(df[col], errors= 'coerce')

date_cols = ['time','time_race','race_start','fp1_date' , 'fp2_date','fp3_date'] 
for col in date_cols :
    if col in df.columns:
       try :
           df[col] = pd.to_datetime(df[col], format='%Y-%m-%d', errors='coerce')
       except:
           df[col] = pd.to_datetime(df[col], format='%Y-%m-%d %H:%M:%S', errors='coerce')

time_cols = ['fastestLapTime']
for col in time_cols:
    if col in df.columns:
        df[col] = pd.to_timedelta(df[col], errors='coerce')

#remove nulls (numericals) per row with more than "threshold" missing values
threshold  = 4        #threshold to decide removing or imputating
df=df[df.isnull().sum(axis=1) < threshold]    

#replace empty numericals with mean value of their col 
excluded = ["resultId", "raceId", "driverId", "constructorId", "statusId"] #Ids are constants 
changable_numerial_cols = [col for col in numerical_cols if col not in excluded]
for col in changable_numerial_cols :
    mean_value = df[col].mean()
    df[col] = df[col].fillna(mean_value)     

#removes rows with any missing text values
text_cols = df.select_dtypes(include=['object']).columns
df = df[df[text_cols].notnull().all(axis=1)]

#Remove rows with missing texts
text_cols = df.select_dtypes(include=['object']).columns 
for col in text_cols:
    df[col] = df[col].str.lower().str.strip()  #lowercasing all characters
df = df[df[text_cols].notnull().all(axis=1)]

#remove dupliactes exceluding the id -automatically generated-
df = df.drop_duplicates(subset=df.columns.difference(['resultId'])) 

#reset index
df = df.reset_index(drop=True)

#Saving cleaned data
df.to_csv("cleaned_results.csv", index=False, quoting=csv.QUOTE_MINIMAL)

#------------------
#Prepartion 
#------------------

#clipping exterme values 
for col in changable_numerial_cols:
    lower = df[col].quantile(0.05)
    upper = df[col].quantile(0.95)
    df[col] = df[col].clip(lower, upper)

#--------------------
#Engineering features 
#--------------------

#Extra_Cols
df["positions_lost"] = df["grid"] - df["positionOrder"]
df["dnf"] = ~df["status"].str.contains("finished" , na=False) #the times the driver didn't finish

#-------
#Driver
#-------

#Driver_finish_rate
driver_finish_rate = df.groupby("driverId")["dnf"].apply(lambda x: 1 - x.mean())            #1 - p(didn't finish) #grouped by driverId
df = df.merge(driver_finish_rate.rename("driver_finish_rate"), on="driverId", how="left")
df.to_csv("prepered_results.csv", index=False, quoting=csv.QUOTE_MINIMAL)

#driver_Points per Season
driver_per_season = df.groupby(["year","driverId"])["points"].sum().reset_index()
driver_per_season.rename(columns={"points" : "driver_season_points"}, inplace= True)
df = df.merge(driver_per_season, on= ["year" , "driverId"] , how="left")

#driver total points
driver_points = df.groupby("driverId")["points"].sum().reset_index()
driver_points.rename(columns={"points" : "driver_career_points"}, inplace= True)
df = df.merge(driver_points , how= "left")

#driver average points
driver_race_counts = df.groupby("driverId")["raceId"].count().reset_index()
driver_race_counts.rename(columns={"raceId": "driver_race_count"}, inplace=True)
df = df.merge(driver_race_counts, on="driverId", how="left")
df["driver_avg_points_per_race"] = df["driver_career_points"] / df["driver_race_count"]

#-----------
#Constructor
#-----------

#Constructor_finish_rate
constructor_finish_rate = df.groupby("constructorId")["dnf"].apply(lambda x: 1 - x.mean())  #grouped by constructorId
df = df.merge(constructor_finish_rate.rename("constructor_finish_rate"), on="constructorId", how="left")

#Constructor_points by season
constructor_season_points = df.groupby(["year", "constructorId"])["points"].sum().reset_index()
constructor_season_points.rename(columns= {"points" : "constructor_season_points"}, inplace= True) 
df = df.merge(constructor_season_points,on= ["year" , "constructorId"],how = "left")

#Constructor total points
constructor_points = df.groupby("constructorId")["points"].sum().reset_index()
constructor_points.rename(columns={"points" : "constructor_career_points"}, inplace= True)
df = df.merge(constructor_points , how= "left")


#Constructor_points per start
constructor_race_counts = df.groupby("constructorId")["raceId"].count().reset_index()
constructor_race_counts.rename(columns={"raceId": "constructor_race_count"}, inplace=True)
df = df.merge(constructor_race_counts, on="constructorId", how="left")
print(constructor_race_counts) #to check if some teams are biased
df["constructor_avg_points_per_race"] = df["constructor_career_points"] / df["constructor_race_count"]

#Decade
df["decade"] = (df["year"] // 10) * 10
decade_total_points = df.groupby("decade")["points"].sum().reset_index()            #total points
decade_total_points.rename(columns={"points" : "decade_total_points"},inplace=True)
df = df.merge(decade_total_points, on="decade",how="left")


decade_total_dnfs = df[df["dnf"] == True].groupby("decade")["dnf"].sum().reset_index() #total dnfs
decade_total_dnfs.rename(columns={"dnf": "total_dnfs"}, inplace=True)

decade_total_races = df.groupby("decade")["raceId"].count().reset_index()
decade_total_races.rename(columns= {"raceId" : "Total_races"}, inplace=True)
df = df.merge(decade_total_races, on="decade",how="left")

df["decade_avg_points_per_race"] = df["decade_total_points"] / df["Total_races"]
df = df.merge(decade_total_dnfs, on="decade", how="left")

##################
df.to_csv("prepered_results.csv", index=False, quoting=csv.QUOTE_MINIMAL)



#--------------------
#Findings 
#--------------------

#Top 10 drivers by total points
top_driver = df[["driverId", "driver_career_points"]].drop_duplicates()\
.sort_values("driver_career_points", ascending=False).head(10)
top_driver.insert(0, "Rank", range(1, 11))
top_driver.columns = ["Rank", "Driver ID", "Total Points"]

#Top 10 consistent players 
top_avg = df[["driverId", "driver_avg_points_per_race"]].drop_duplicates()\
.sort_values("driver_avg_points_per_race", ascending=False).head(10)

top_avg.insert(0, "Rank", range(1, 11))
top_avg.columns = ["Rank", "Driver ID", "Avg Points Per Race"]

#Exporting to Csv
top_driver.to_csv("Top_drivers.csv" , index= False)
top_driver.to_csv("Most consistent players.csv" , index= False)

#Teams most points per start
team_eff = (
    df[["constructorId", "constructor_avg_points_per_race"]]
    .drop_duplicates()\
    .sort_values("constructor_avg_points_per_race", ascending=False).head(10)
)
team_eff.to_csv("team_efficiency.csv", index=False)

#Which season had the most DNFs,
dnf_per_year = df[df["dnf"] == True].groupby("year").size().reset_index(name="dnf_count") #size counts even non nulls
most_dnf_year = dnf_per_year.sort_values("dnf_count", ascending=False).iloc[0]          

most_dnf_year_data = df[(df["dnf"] == True) & (df["year"] == most_dnf_year["year"])] #df for all dnfs == true in the year with most
dnf_causes = (
    most_dnf_year_data["status"].value_counts() .reset_index().head(15)
)
dnf_causes.columns = ["cause", "count"]

dnf_causes.to_csv("dangerous_year.csv", index=False)

#Worst Performers
driver_lost_positions = (
    df.groupby("driverId")["positions_lost"].sum().reset_index()\
    .sort_values(by="positions_lost", ascending=False).head(10)
)
driver_lost_positions.to_csv("worst_performers.csv", index=False)

#decade comparison
decade_summary = df[["decade", "total_dnfs", "decade_total_points", "Total_races", "decade_avg_points_per_race"]]
decade_summary = decade_summary.drop_duplicates(subset=["decade"]).sort_values("decade")
decade_summary.to_csv("decade_summary.csv", index=False)


#-------------------
#visualization          #Styled with help of Ai -- picked the graphs myself
#-------------------
#Top 10 players
plt.figure(figsize=(10, 6))
plt.barh(
    top_driver["Driver ID"].astype(str),
    top_driver["Total Points"]
)
plt.gca().invert_yaxis()                    # highest-scoring at the top
plt.xlabel("Total Career Points")
plt.ylabel("ID")
plt.title("Top 10 Drivers by Total Points")
plt.tight_layout()
plt.savefig("top_10_drivers_by_points.png", dpi=300)

#Top 10 Most Consistent Drivers 
plt.figure(figsize=(10, 6))
plt.barh(
    top_avg["Driver ID"].astype(str),
    top_avg["Avg Points Per Race"],
    color="seagreen"
)
plt.gca().invert_yaxis()
plt.xlabel("Average Points per Race")
plt.ylabel("ID")
plt.title("Top 10 Most Consistent Drivers")
plt.tight_layout()
plt.savefig("top_10_drivers_by_avg_points.png", dpi=300)
plt.show()

#Worst players
plt.figure(figsize=(10, 6))
plt.barh(
    driver_lost_positions["driverId"].astype(str),
    driver_lost_positions["positions_lost"],
    color="crimson"
)
plt.gca().invert_yaxis() 
plt.xlabel("Total Positions Lost")
plt.ylabel("Driver ID")
plt.title("Top 10 Worst Performers by Positions Lost")
plt.tight_layout()
plt.savefig("worst_performers_positions_lost.png", dpi=300)
plt.show()


# Constructors
constructor_details = (
    df[[
        "constructorId",
        "constructor_finish_rate",
        "constructor_career_points",
        "constructor_avg_points_per_race"
    ]]
    .drop_duplicates()
)

# Compute a size scale based on career points
max_pts = constructor_details["constructor_career_points"].max()
constructor_details["scatter_size"] = (constructor_details["constructor_career_points"] / max_pts) * 900  

# Plot
plt.figure(figsize=(10, 6))
plt.scatter(
    constructor_details["constructor_avg_points_per_race"],
    constructor_details["constructor_finish_rate"],
    s=constructor_details["scatter_size"],
    alpha=0.7,
    edgecolors="black"
)

#each point with its constructor ID
for _, r in constructor_details.iterrows():     #doesn't care about the row's index
    plt.text(
        r["constructor_avg_points_per_race"] + 0.02,
        r["constructor_finish_rate"] + 0.005,
        str(int(r["constructorId"])),
        fontsize=12
    )

plt.xlabel("Avg Points Per Race")
plt.ylabel("Finish Rate")
plt.title("Constructor Efficiency vs Reliability")
plt.grid(True, linestyle="--", linewidth=0.6, alpha=0.7)
plt.tight_layout()  
plt.savefig("constructors_efficiency_vs_reliability.png", dpi=300)
plt.show()


#most year with dnfs
plt.figure(figsize=(12, 8))
ax = plt.gca()
ax.bar(dnf_per_year["year"].astype(int), dnf_per_year["dnf_count"], color="tomato")
# Tell the x-axis to use at most 10 integer ticks
ax.xaxis.set_major_locator(MaxNLocator(nbins=10, integer=True))

ax.set_xlabel("Season (Year)")
ax.set_ylabel("Total DNFs")
ax.set_title("Total DNFs per F1 Season")

plt.tight_layout()
plt.savefig("dnfs_per_year.png", dpi=200)
plt.figure(figsize=(8, 8))
plt.pie(
    dnf_causes["count"],
    labels=dnf_causes["cause"],
    autopct='%1.1f%%',
    startangle=140
)
plt.title(f"Top DNF Causes in {int(most_dnf_year['year'])}")
plt.axis("equal")
plt.tight_layout()
plt.savefig("dnf_causes_pie_chart.png", dpi=300)
plt.show()


#Decades
years = decade_summary["decade"].astype(int).astype(str)

fig, axes = plt.subplots(2, 2, figsize=(14, 10), sharex=True)

# 1) Total Points per Decade
axes[0, 0].bar(years, decade_summary["decade_total_points"], color="steelblue")
axes[0, 0].set_title("Total Points per Decade")
axes[0, 0].set_ylabel("Total Points")

# 2) Total DNFs per Decade
axes[0, 1].bar(years, decade_summary["total_dnfs"], color="tomato")
axes[0, 1].set_title("Total DNFs per Decade")
axes[0, 1].set_ylabel("DNFs")

# 3) Total Races per Decade
axes[1, 0].bar(years, decade_summary["Total_races"], color="forestgreen")
axes[1, 0].set_title("Total Races per Decade")
axes[1, 0].set_ylabel("Races")

# 4) Avg Points per Race per Decade
axes[1, 1].bar(years, decade_summary["decade_avg_points_per_race"], color="mediumpurple")
axes[1, 1].set_title("Avg Points/Race per Decade")
axes[1, 1].set_ylabel("Avg Points/Race")

# Common tweaks
for ax in axes.flat:
    ax.tick_params(axis='x', rotation=45)

plt.suptitle("F1 Decade-by-Decade Summary", fontsize=16)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.savefig("decade_overview_4panels.png", dpi=300)
plt.show()