package com.example.taxi.ui.home.user

import android.os.Bundle
import android.util.Log
import android.view.View
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.databinding.FragmentUserHomeBinding
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class UserHomeFragment: BaseFragment<FragmentUserHomeBinding>(R.layout.fragment_user_home) {
    private lateinit var destinationListAdapter: DestinationListAdapter
    private lateinit var favoritesAdapter: FavoritesAdapter
    private val userHomeViewModel : UserHomeViewModel by viewModels()

    private val favoritesDeleteClickListener: (View, String) -> Unit = { _, address ->
        showFavoritesDialog(address)
    }

    override fun init() {
        initAdapter()
        observerData()
        setOnClickListeners()
    }

    private fun initAdapter() {
        userHomeViewModel.getDestinations()
        destinationListAdapter = DestinationListAdapter()
        binding.recyclerviewUserHomeDestinationList.layoutManager = LinearLayoutManager(requireContext())
        binding.recyclerviewUserHomeDestinationList.adapter = destinationListAdapter
        userHomeViewModel.getFavorites()
        favoritesAdapter = FavoritesAdapter().apply {
            onItemClickListener = favoritesDeleteClickListener
        }
        binding.recyclerviewUserHomeFavorites.apply {
            adapter = favoritesAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
    }

    private fun observerData() {
        userHomeViewModel.destinations.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                    binding.recyclerviewUserHomeDestinationList.setBackgroundResource(R.drawable.layout_recycler_no_item)
                    binding.textUserHomeNoContentDestination.show()
                }
                is UiState.Success -> {
                    //binding.progressBar.hide()
                    val list : MutableList<Destination> = state.data.toMutableList()
                    destinationListAdapter.updateList(list)
                    binding.recyclerviewUserHomeDestinationList.setBackgroundResource(R.drawable.layout_recycler)
                    binding.textUserHomeNoContentDestination.hide()
                }
            }
        }
        userHomeViewModel.favorites.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                    binding.recyclerviewUserHomeFavorites.setBackgroundResource(R.drawable.layout_recycler_no_item)
                    binding.textUserHomeNoContentFavorites.show()
                }
                is UiState.Success -> {
                    //binding.progressBar.hide()
                    val list : MutableList<Favorites> = state.data.toMutableList()
                    favoritesAdapter.updateList(list)
                    binding.recyclerviewUserHomeFavorites.setBackgroundResource(R.drawable.layout_recycler)
                    binding.textUserHomeNoContentFavorites.hide()
                }
            }
        }
    }

    private fun setOnClickListeners(){
        binding.buttonUserHomeCallTaxi.setOnClickListener {
            findNavController().navigate(R.id.action_userHomeFragment_to_destinationSettingFragment)
        }
    }

    private fun showFavoritesDialog(address: String) {
        FavoritesDialogFragment { favoritesListener(address) }.show(childFragmentManager, "FAVORITES_DIALOG")
    }

    private val favoritesListener: (address: String) -> Unit = {
        //userHomeViewModel.deleteFavorites()
    }
}