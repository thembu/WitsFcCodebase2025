import numpy as np


def role_assignment(teammate_positions, formation_positions):
   
    
    # Initialize the output dictionary
    point_preferences = {}
    
    # Step 1: Identify valid players (those with non-None positions)
    valid_players = []
    for player_idx in range(len(teammate_positions)):
        if teammate_positions[player_idx] is not None:
            valid_players.append(player_idx)
    
    # If no valid players, return empty dictionary
    if len(valid_players) == 0:
        return point_preferences
    
    # Step 2: Create preference lists for players based on distance to roles
    player_preferences = {}
    
    for player_idx in valid_players:
        distances = []
        player_pos = np.array(teammate_positions[player_idx])
        
        for role_idx in range(len(formation_positions)):
            role_pos = np.array(formation_positions[role_idx])
            # Calculate Euclidean distance
            dist = np.linalg.norm(player_pos - role_pos)
            distances.append((role_idx, dist))
        
        # Sort by distance (closest first) and create preference list
        distances.sort(key=lambda x: x[1])
        player_preferences[player_idx] = [role_idx for role_idx, _ in distances]
    
    # Step 3: Create preference lists for roles based on distance to players
    role_preferences = {}
    
    for role_idx in range(len(formation_positions)):
        distances = []
        role_pos = np.array(formation_positions[role_idx])
        
        for player_idx in valid_players:
            player_pos = np.array(teammate_positions[player_idx])
            # Calculate Euclidean distance
            dist = np.linalg.norm(role_pos - player_pos)
            distances.append((player_idx, dist))
        
        # Sort by distance (closest first) and create preference list
        distances.sort(key=lambda x: x[1])
        role_preferences[role_idx] = [player_idx for player_idx, _ in distances]
    
    # Step 4: Run Gale-Shapley Algorithm
    # Players "propose" to roles, roles accept or reject based on preferences
    
    unmatched_players = list(valid_players)
    current_matches = {}  # role_idx -> player_idx
    next_proposal = {player_idx: 0 for player_idx in valid_players}  # tracks next role to propose to
    
    while unmatched_players:
        player_idx = unmatched_players.pop(0)
        
        # Check if player has exhausted all preferences
        if next_proposal[player_idx] >= len(player_preferences[player_idx]):
            continue
        
        # Get the next role this player will propose to
        role_idx = player_preferences[player_idx][next_proposal[player_idx]]
        next_proposal[player_idx] += 1
        
        # If role is unmatched, match them
        if role_idx not in current_matches:
            current_matches[role_idx] = player_idx
        else:
            # Role is already matched, check if it prefers this player
            current_player = current_matches[role_idx]
            role_pref_list = role_preferences[role_idx]
            
            # Find positions in preference list
            new_player_rank = role_pref_list.index(player_idx)
            current_player_rank = role_pref_list.index(current_player)
            
            # If role prefers new player over current match
            if new_player_rank < current_player_rank:
                # Switch matches
                current_matches[role_idx] = player_idx
                unmatched_players.append(current_player)
            else:
                # Keep current match, new player tries again
                unmatched_players.append(player_idx)
    
   
    for role_idx, player_idx in current_matches.items():
        unum = player_idx + 1 
        point_preferences[unum] = np.array(formation_positions[role_idx])
    
    return point_preferences