package com.example.taxi.di

import android.content.SharedPreferences
import com.example.taxi.data.repository.*
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.firestore.FirebaseFirestore
import com.google.gson.Gson
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object RepositoryModule {
    @Provides
    @Singleton
    fun provideFrequentDestinationRepository(
        database: FirebaseFirestore
    ): FrequentDestinationRepository {
        return FrequentDestinationRepositoryImpl(database)
    }

    @Provides
    @Singleton
    fun provideAutghRepository(
        database: FirebaseFirestore,
        auth: FirebaseAuth,
        appPreferences: SharedPreferences,
        gson: Gson
    ): AuthRepository {
        return AuthRepositoryImpl(auth,database,appPreferences,gson)
    }

    @Provides
    @Singleton
    fun provideRouteRepository(
        database: FirebaseFirestore
    ): RouteRepository {
        return RouteRepositoryImpl(database)
    }

    @Provides
    @Singleton
    fun userInfoRepository(
        database: FirebaseFirestore
//        storage: FirebaseStorage
    ) : UserInfoRepository{
        return UserInfoRepositoryImpl(database)
//        return UserInfoRepositoryImpl(database, storage)
    }
}