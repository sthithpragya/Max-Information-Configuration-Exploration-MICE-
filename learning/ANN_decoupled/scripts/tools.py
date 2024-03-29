import torch
import torch.nn.functional as F
import itertools
import custom_models
import yaml

with open('./../param.yaml',"r") as learningStream:
	paramLoaded = yaml.safe_load(learningStream)

totalJoints = paramLoaded["totalJoints"]

def grid_search_loop(train_data, test_data, device, seeds, nb_folds, eta_grid, 
                     nb_hidden_layers_grid, nb_hidden_neurons_grid, debug, jointIndex):

    # Percentage of data used for validation
    val_ratio = 0.2

    # Number of epochs
    nb_epochs = 100
    
    # Mini-batch size
    batch_size = 12000

    # Compute number of hyperparameters combinations
    nb_combinations = len(eta_grid) * len(nb_hidden_layers_grid) * \
                      len(nb_hidden_neurons_grid)

    # Create a 2D grid to store hyperparameters, classification performances 
    # and the number of parameters in the model
    results_grid = torch.empty(nb_combinations, 10).zero_()

    # Iterator of the parameters
    param_generator = iter(itertools.product(eta_grid, nb_hidden_layers_grid,
                                             nb_hidden_neurons_grid))
    
    # Initialize combinations' counter to zero
    count = 0
    
    while True:
        try:
            # Extract hyperparameters' combination
            param_list = next(param_generator)
            [eta, nb_hidden_layers, nb_hidden_neurons] = param_list

            # Create empty tensors to save the classification performance of 
            # the model on the training, validation and testing sets
            result_train = torch.empty(len(seeds), nb_folds).to(device)
            result_val = torch.empty(len(seeds), nb_folds).to(device)
            result_test = torch.empty(len(seeds), nb_folds).to(device)

            # torch.cuda.clear_memory_allocated()  # entirely clear all allocated memory

            # Transfer the tensors to the CPU or GPU
            train_data = train_data.to(device)
            test_data = test_data.to(device)

            # Loop over the seeds
            for s in range(len(seeds)):
                # Set seed
                torch.manual_seed(seeds[s])
                torch.random.manual_seed(seeds[s])
 
                # Loop over the folds
                for f in range(0, nb_folds):
                    # Split the train sets into training/validation sets
                    idx = torch.randperm(train_data.size(0))
                    n_val = int(val_ratio * train_data.size(0))
                    val_input = train_data[idx[0:n_val], 0:3*totalJoints]
                    val_target = train_data[idx[0:n_val], 3*totalJoints+jointIndex].unsqueeze(1)
                    train_input = train_data[idx[n_val:], 0:3*totalJoints]
                    train_target = train_data[idx[n_val:], 3*totalJoints+jointIndex].unsqueeze(1)
                    
                    idx = torch.randperm(test_data.size(0))
                    n_val = 20000
                    test_input = test_data[idx[0:n_val], 0:3*totalJoints]
                    test_target = test_data[idx[0:n_val], 3*totalJoints+jointIndex].unsqueeze(1)
                    
                      # Create ResNet-based model
                    model = custom_models.InverseDynamicModel(nb_hidden_layers,
                                                nb_hidden_neurons).to(device)

                    # Train model
                    train_model(model, train_input, train_target, nb_epochs, 
                                batch_size, eta, s, len(seeds), f, nb_folds, 
                                debug)

                    # Compute train error
                    # print(train_input.size())
                    
                    train_error = compute_loss(model, train_input, train_target)
                    
                    result_train[s, f] = train_error

                    # Compute val error
                    val_error = compute_loss(model, val_input, val_target)

                    result_val[s, f] = val_error

                    # Compute test error
                    test_error = compute_loss(model, test_input, test_target)
    
                    result_test[s, f] = test_error
                    
            # Compute number of parameters in the model
            total_nb_parameters = 0
            for p in model.parameters():
                total_nb_parameters += p.nelement()

            # Fill results' grid
            results_grid[count, 0] = eta
            results_grid[count, 1] = nb_hidden_layers
            results_grid[count, 2] = nb_hidden_neurons
            results_grid[count, 3] = result_train.mean()
            results_grid[count, 4] = result_train.std()
            results_grid[count, 5] = result_val.mean()
            results_grid[count, 6] = result_val.std()
            results_grid[count, 7] = result_test.mean()
            results_grid[count, 8] = result_test.std()
            results_grid[count, 9] = total_nb_parameters
            
            count += 1
            print('Combinations tested: {:d}/{:d}' \
                  .format(count, nb_combinations) + ' | Current Combination:' + str(param_list))

        except StopIteration:
            return results_grid


def train_model(model, train_input, train_target, num_epochs, mini_batch_size, 
                eta, seed, nb_seeds, fold, nb_folds, debug):

    
    optimizer = torch.optim.Adam(model.parameters(), lr=eta)

    # Loop over the epochs
    for epoch in range(num_epochs):
        # Loop over the batches
        for b in range(0, train_input.size(0), mini_batch_size):
            # Extract mini-batch input, target
            x = train_input.narrow(0, b, mini_batch_size)
            t = train_target.narrow(0, b, mini_batch_size)
    
            # Compute mini-batch output
            y = model(x)
    
            # Compute mini-batch loss
            loss = F.mse_loss(y, t)
    
            # Set model parameter gradients to zero
            model.zero_grad()
    
            # Back-propagate gradient
            loss.backward()
    
            # Update parameters
            optimizer.step()

            # Print epoch, batch and loss information
            if debug:
                print(("Seed: %03d/%03d | Fold:%03d/%03d | Epoch: %03d/%03d |"
                      + " Batch %03d/%03d | Loss: %.4f") % (seed + 1, nb_seeds, 
                      fold + 1, nb_folds, epoch + 1, num_epochs, b, 
                      train_input.size(0),  loss))

def compute_loss(model, input_data, target_data):
  with torch.no_grad():
    return F.mse_loss(model(input_data), target_data)
