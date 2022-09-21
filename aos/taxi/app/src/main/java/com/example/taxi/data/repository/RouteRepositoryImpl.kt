package com.example.taxi.data.repository

import android.util.Log
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.data.dto.user.route.Location
import com.example.taxi.data.dto.user.route.Route
import com.example.taxi.data.dto.user.route.RouteSetting
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.FireStoreCollection
import com.example.taxi.utils.constant.UiState
import com.google.firebase.firestore.FirebaseFirestore

class RouteRepositoryImpl(
    private val database: FirebaseFirestore
) : RouteRepository {
    override fun getRoute(result: (UiState<List<Location>>) -> Unit) {
        database.collection(FireStoreCollection.ROUTE).document("12가3456")
            .get()
            .addOnSuccessListener { document ->
                Log.d("getRoute", "DocumentSnapshot data: ${document.data}")
                if (document != null) {
                    val route = document.toObject(Route::class.java)
                    if(route != null){
                        val locations = route.position
                        val newLocation = mutableListOf<Location>()
                        if(locations != null){
                            for(des in locations){
                                val location = Location(des.long, des.lati)
                                newLocation.add(location)
                            }
                            result.invoke(
                                UiState.Success(newLocation)
                            )
                        }
                    }
                } else {
                    Log.d("getRoute", "No such document")
                }
            }
            .addOnFailureListener {
                Log.d("getRoute", "Fail get document")
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
            }
    }

    override fun addRouteSetting(
        routeSetting: RouteSetting,
        result: (UiState<RouteSetting>) -> Unit
    ) {
        val document = database.collection(FireStoreCollection.ROUTESETTING).document("12가3456")
        document
            .set(routeSetting)
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success(routeSetting)
                )
                Log.d("addRouteSetting", "Destination has been created successfully")
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
                Log.d("addRouteSetting", "Destination has been created fail")
            }
    }
}