
list_of_genres = ["electaronic","pop"]
genre = ""

for genre_name in list_of_genres:
    if "soul" in genre_name:
        genre = "soul"
    if "pop" in genre_name:
        genre = "pop"
    if "rock" in genre_name:
        genre = "rock"
    if "metal" in genre_name:
        genre = "metal"
    if "blues" in genre_name:
        genre = "blues"
    if "classical" in genre_name:
        genre = "classical"
    if "electr" in genre_name:
        genre = "electr"
    # If a genre has been found, doesn't need to iterate through the other genres
    if genre != "":
        break

print(genre)
print(len(list_of_genres))