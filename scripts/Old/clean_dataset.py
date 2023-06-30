import csv

def simplify_database(file_name):
    with open(file_name, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        output_data = []

        for row in reader:
            trial = row["TRIAL"]
            customer_speech = row["ACTUAL CUSTOMER SPEECH"].strip()
            shopkeeper_speech = row["ACTUAL SHOPKEEPER SPEECH"].strip()
            
            # Only add a row if the customer or shopkeeper said something
            if customer_speech != "NONE":
                output_data.append({"TRIAL": trial, "CONVERSATION": customer_speech})

            if shopkeeper_speech != "NONE":
                output_data.append({"TRIAL": trial, "CONVERSATION": shopkeeper_speech})
    
    # Write the simplified data to a CSV file
    with open('simplified_database.csv', 'w', newline='') as csvfile:
        fieldnames = ['TRIAL', 'CONVERSATION']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for data in output_data:
            writer.writerow(data)


output_data = simplify_database("/home/victor/catkin_ws/src/visual_ai4hri/scripts/LearningProactiveDataset.csv")

